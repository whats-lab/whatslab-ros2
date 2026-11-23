SIDES = ('left', 'right')
NAMESPACE = '/airglove'


def joints_topic(side: str) -> str:
    return f'{NAMESPACE}/{side}/joints'


def wrist_topic(side: str) -> str:
    return f'{NAMESPACE}/{side}/wrist'


def haptics_topic(side: str) -> str:
    return f'{NAMESPACE}/{side}/haptics'


def haptics_result_topic(side: str) -> str:
    return f'{NAMESPACE}/{side}/haptics_result'


STATUS_TOPIC = f'{NAMESPACE}/status'
ALARM_TOPIC = f'{NAMESPACE}/alarm'


def wrist_frame(side: str) -> str:
    return f'airglove_{side}_wrist'
