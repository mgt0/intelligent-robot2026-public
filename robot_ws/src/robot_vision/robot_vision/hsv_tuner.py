"""HSV 범위 보정 도구 (ROS 불필요).
사용: hsv_tuner <이미지|영상 파일|카메라 번호> [--init H0 S0 V0 H1 S1 V1]
q 또는 ESC로 종료하면 값을 출력한다."""
import argparse

import cv2
import numpy as np

NAMES = ['H low', 'S low', 'V low', 'H high', 'S high', 'V high']


def main(args=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('source', help='이미지/영상 파일 경로 또는 카메라 번호')
    ap.add_argument('--init', type=int, nargs=6, default=[18, 80, 80, 35, 255, 255])
    a = ap.parse_args(args)

    still, cap = None, None
    if a.source.isdigit():
        cap = cv2.VideoCapture(int(a.source))
    else:
        still = cv2.imread(a.source)
        if still is None:
            cap = cv2.VideoCapture(a.source)
    if still is None and not cap.isOpened():
        raise SystemExit(f'열 수 없음: {a.source}')

    win = 'hsv_tuner'
    cv2.namedWindow(win)
    for n, v, mx in zip(NAMES, a.init, [179, 255, 255, 179, 255, 255]):
        cv2.createTrackbar(n, win, v, mx, lambda _: None)

    vals = list(a.init)
    while True:
        if still is not None:
            frame = still
        else:
            ok, frame = cap.read()
            if not ok and a.source.isdigit():
                break
            if not ok:  # 영상 파일은 처음부터 반복
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
        vals = [cv2.getTrackbarPos(n, win) for n in NAMES]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(vals[:3], np.uint8), np.array(vals[3:], np.uint8))
        view = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)])
        cv2.imshow(win, view)
        if cv2.waitKey(30) & 0xFF in (ord('q'), 27):
            break
    print(f'hsv_lower: {vals[:3]}\nhsv_upper: {vals[3:]}')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
