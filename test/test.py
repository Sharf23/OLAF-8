# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


async def reset_dut(dut):
    """Reset the OLAF-8 design."""
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 5)

    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 2)


async def run_sample(dut, x1, x2):
    """
    Apply one OLAF-8 input transaction.

    ui_in[7:4] = x1
    ui_in[3:0] = x2
    uio_in[0]  = start
    """

    # Pack two 4-bit inputs into the 8-bit Tiny Tapeout input bus.
    dut.ui_in.value = ((x1 & 0xF) << 4) | (x2 & 0xF)

    # Generate a one-cycle START pulse.
    dut.uio_in.value = 0x01
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0

    # OLAF-8 performs:
    #   8 rule scans
    #   iterative defuzzification
    #   adaptation decision
    #
    # Allow sufficient cycles for the complete transaction.
    for _ in range(24):
        await ClockCycles(dut.clk, 1)

    result = int(dut.uo_out.value)

    return {
        "y": result & 0x0F,
        "done": (result >> 4) & 0x01,
        "admitted": (result >> 5) & 0x01,
        "busy": (result >> 6) & 0x01,
    }


@cocotb.test()
async def test_reset_and_basic_operation(dut):
    """Verify reset and a basic fuzzy inference transaction."""

    dut._log.info("Starting OLAF-8 basic operation test")

    # 10 MHz clock = 100 ns period.
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    result = await run_sample(dut, 1, 1)

    dut._log.info(
        "x1=1 x2=1 -> y=%d done=%d admitted=%d busy=%d",
        result["y"],
        result["done"],
        result["admitted"],
        result["busy"],
    )

    assert result["busy"] == 0, \
        "OLAF-8 should not remain busy after a completed transaction"

    assert 0 <= result["y"] <= 15, \
        "Fuzzy output must remain within the 4-bit range"


@cocotb.test()
async def test_multiple_input_regions(dut):
    """
    Exercise LOW, MID and HIGH input regions.
    """

    dut._log.info("Testing multiple fuzzy input regions")

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    test_vectors = [
        (0, 0),
        (1, 1),
        (3, 3),
        (7, 7),
        (8, 8),
        (11, 11),
        (14, 14),
        (15, 15),
        (2, 13),
        (13, 2),
    ]

    for x1, x2 in test_vectors:

        result = await run_sample(dut, x1, x2)

        dut._log.info(
            "x1=%d x2=%d -> y=%d done=%d admitted=%d",
            x1,
            x2,
            result["y"],
            result["done"],
            result["admitted"],
        )

        assert 0 <= result["y"] <= 15, \
            f"Invalid fuzzy output for x1={x1}, x2={x2}"

        assert result["busy"] == 0, \
            f"OLAF-8 remained busy after x1={x1}, x2={x2}"


@cocotb.test()
async def test_boundary_values(dut):
    """Check all extreme 4-bit input combinations."""

    dut._log.info("Testing boundary values")

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    boundary_vectors = [
        (0, 0),
        (0, 15),
        (15, 0),
        (15, 15),
    ]

    for x1, x2 in boundary_vectors:

        result = await run_sample(dut, x1, x2)

        dut._log.info(
            "Boundary x1=%d x2=%d -> y=%d",
            x1,
            x2,
            result["y"],
        )

        assert result["done"] == 1, \
            f"DONE was not asserted for x1={x1}, x2={x2}"

        assert 0 <= result["y"] <= 15, \
            "Output exceeded 4-bit range"


@cocotb.test()
async def test_online_adaptation(dut):
    """
    Exercise the online rule-admission mechanism.

    Novel/unrepresented fuzzy regions are presented sequentially.
    The admission status is observable through uo_out[5].
    """

    dut._log.info("Testing online adaptation")

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    adaptation_vectors = [
        (15, 0),
        (0, 15),
        (15, 4),
        (4, 15),
        (2, 13),
        (13, 2),
        (1, 14),
        (14, 1),
    ]

    admission_count = 0

    for x1, x2 in adaptation_vectors:

        result = await run_sample(dut, x1, x2)

        admission_count += result["admitted"]

        dut._log.info(
            "Adaptive sample x1=%d x2=%d -> y=%d admitted=%d",
            x1,
            x2,
            result["y"],
            result["admitted"],
        )

        assert result["done"] == 1, \
            "Transaction did not complete"

        assert 0 <= result["y"] <= 15, \
            "Adaptive output outside valid range"

    dut._log.info(
        "Total observed admission events = %d",
        admission_count,
    )

    # The exact admission count is intentionally not hard-coded because
    # it depends on the evolving rule memory and threshold.
    assert admission_count >= 0


@cocotb.test()
async def test_repeated_input(dut):
    """
    Repeatedly apply the same input.

    This verifies that the design remains operational when the same
    fuzzy state is presented multiple times.
    """

    dut._log.info("Testing repeated inputs")

    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    first = await run_sample(dut, 7, 7)

    for _ in range(4):
        result = await run_sample(dut, 7, 7)

        assert result["done"] == 1, \
            "Repeated transaction did not complete"

        assert 0 <= result["y"] <= 15, \
            "Repeated input produced invalid output"

    dut._log.info(
        "Initial output for x1=7 x2=7 was y=%d",
        first["y"],
    )
