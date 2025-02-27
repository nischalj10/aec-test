import pytest
from pytest_embedded import Dut


@pytest.mark.target('esp32s3')
@pytest.mark.env('esp32s3')
@pytest.mark.parametrize(
    'config',
    [
        'mn5q8_cn',
        'mn5q8_en',
        'mn6_cn',
        'mn6_en',
        'mn7_en',
    ],
)
def test_multinet_s3(dut: Dut)-> None:
    dut.run_all_single_board_cases(group="mn")

@pytest.mark.target('esp32p4')
@pytest.mark.env('esp32p4')
@pytest.mark.parametrize(
    'config',
    [
        'p4_mn7_en',
        'p4_mn7_cn',
    ],
)
def test_multinet_p4(dut: Dut)-> None:
    dut.run_all_single_board_cases(group="mn")


@pytest.mark.target('esp32s3')
@pytest.mark.env('esp32s3')
@pytest.mark.parametrize(
    'config',
    [
        'mn5q8_en',
        'wn9_hilexin',
    ],
)
def test_wakenet(dut: Dut)-> None:
    dut.run_all_single_board_cases(group="wn")

@pytest.mark.target('esp32p4')
@pytest.mark.env('esp32p4')
@pytest.mark.parametrize(
    'config',
    [
        'p4_mn7_en',
        'p4_nsnet2',
    ],
)
def test_wakenet_p4(dut: Dut)-> None:
    dut.run_all_single_board_cases(group="wn")

@pytest.mark.target('esp32s3')
@pytest.mark.env('esp32s3')
@pytest.mark.parametrize(
    'config',
    [
        'wn9_hilexin',
        'vadnet',
    ],
)
def test_sr_afe(dut: Dut)-> None:
    dut.run_all_single_board_cases(group="afe_sr", timeout=100000)

@pytest.mark.target('esp32p4')
@pytest.mark.env('esp32p4')
@pytest.mark.parametrize(
    'config',
    [
        'p4_mn7_cn',
    ],
)
def test_sr_afe_p4(dut: Dut)-> None:
    dut.run_all_single_board_cases(group="afe_sr", timeout=100000)


@pytest.mark.target('esp32s3')
@pytest.mark.env('esp32s3')
@pytest.mark.parametrize(
    'config',
    [
        'nsnet2',
    ],
)
def test_vc_afe(dut: Dut)-> None:
    dut.run_all_single_board_cases(group="afe_vc", timeout=100000)


@pytest.mark.target('esp32p4')
@pytest.mark.env('esp32p4')
@pytest.mark.parametrize(
    'config',
    [
        'p4_nsnet2',
    ],
)
def test_vc_afe_p4(dut: Dut)-> None:
    dut.run_all_single_board_cases(group="afe_vc", timeout=100000)