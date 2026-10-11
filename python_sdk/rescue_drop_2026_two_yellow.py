"""救援投放实飞测试入口：红、蓝、绿各一件，同一黄色目标两件。

沿用 rescue_drop_2026.py 的航迹、视觉、避障、继电器和安全退出流程。
真实飞行仍需 --confirm-flight、三个颜色配额参数和终端 start_mission。
"""

import sys

import rescue_drop_2026 as rescue


FREE_DROP_COUNT = 3
MANDATORY_DROP_COUNT = 2
TOTAL_DROP_COUNT = FREE_DROP_COUNT + MANDATORY_DROP_COUNT


def validate_test_allocation(red: int, blue: int, green: int):
    if any(type(count) is not int or count != 1 for count in (red, blue, green)):
        raise ValueError("this test requires --red-count 1 --blue-count 1 --green-count 1")
    return {"red": red, "blue": blue, "green": green}


class TwoYellowMission(rescue.Mission):
    """沿用主任务的双件偏置中点和低空连续投放流程。"""


def main() -> int:
    # 仅在本测试入口进程中覆盖投放配额；主入口文件及其默认行为不变。
    rescue.FREE_DROP_COUNT = FREE_DROP_COUNT
    rescue.MANDATORY_DROP_COUNT = MANDATORY_DROP_COUNT
    rescue.TOTAL_DROP_COUNT = TOTAL_DROP_COUNT
    rescue.validate_allocation = validate_test_allocation
    rescue.Mission = TwoYellowMission
    return rescue.main()


if __name__ == "__main__":
    sys.exit(main())
