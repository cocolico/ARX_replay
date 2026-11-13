#!/usr/bin/env python3
"""Read-only Orbbec color stream -> stable table-board observation. No robot API."""
import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import pyorbbecsdk as ob

from board_align import DEFAULT_BOARD, Observer, StableSamples, save_observation, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', default='CP0B163000CK')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--max-error', type=float, default=1.5)
    parser.add_argument('--board-height-mm', type=float, default=0,
                        help='Printed board plane height above physical tabletop; default 0 assumes coplanar')
    parser.add_argument('--timeout', type=float, default=600)
    parser.add_argument('--board', type=Path, default=DEFAULT_BOARD)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--show', action='store_true')
    parser.add_argument('--manual-save', action='store_true')
    parser.add_argument('--check-stream', action='store_true', help='Check one color frame without saving calibration')
    args = parser.parse_args()
    if args.manual_save and not args.show:
        parser.error('--manual-save requires --show')
    if args.timeout <= 0 or args.max_error <= 0 or not np.isfinite(args.board_height_mm) or args.board_height_mm < 0:
        parser.error('Require positive timeout/error and finite nonnegative board height')
    out = args.out or Path(__file__).resolve().parent / 'results' / datetime.now().strftime('arx_x5_table_%Y%m%d_%H%M%S_%f')
    if out.exists() and any(out.iterdir()):
        parser.error(f'Refusing to overwrite nonempty output directory: {out}')
    context = ob.Context()
    devices = context.query_devices()
    device = None
    for i in range(devices.get_count()):
        candidate = devices.get_device_by_index(i)
        if candidate.get_device_info().get_serial_number() == args.serial:
            device = candidate
            break
    if device is None:
        raise RuntimeError(f'Camera {args.serial} not found')
    pipeline = ob.Pipeline(device)
    profile = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_video_stream_profile(
        args.width, args.height, ob.OBFormat.MJPG, args.fps)
    intr = profile.get_intrinsic()
    dist = profile.get_distortion()
    if str(dist.model) != 'OBCameraDistortionModel.BROWN_CONRADY_K6':
        raise ValueError(f'Unsupported distortion model: {dist.model}')
    c = dict(width=intr.width, height=intr.height,
             K=np.array([[intr.fx, 0, intr.cx], [0, intr.fy, intr.cy], [0, 0, 1.]]),
             D=np.array([getattr(dist, k) for k in ('k1','k2','p1','p2','k3','k4','k5','k6')]),
             model='rational_polynomial', frame_id='gemini335_color_optical_frame')
    observer = Observer(args.board, c, max_error=args.max_error, pose_method='franka_charuco')
    samples = StableSamples(args.samples)
    config = ob.Config()
    config.enable_stream(profile)
    started = False
    try:
        pipeline.start(config)
        started = True
        start = time.monotonic()
        last_log = 0
        last_frame = None
        print(f'Camera {args.serial}: {args.width}x{args.height} MJPG {args.fps} fps. Keep board and camera still.', flush=True)
        while time.monotonic() - start < args.timeout:
            frames = pipeline.wait_for_frames(1000)
            frame = frames.get_color_frame() if frames is not None else None
            if frame is None:
                samples.clear()
                continue
            key = (frame.get_index(), frame.get_timestamp())
            if key == last_frame:
                continue
            last_frame = key
            if frame.get_format() != ob.OBFormat.MJPG:
                raise ValueError('Received unexpected color format')
            image = cv2.imdecode(np.frombuffer(frame.get_data(), dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (c['height'], c['width']):
                raise ValueError('Decoded image does not match stream intrinsics')
            if args.check_stream:
                print(f'Stream OK: {image.shape[1]}x{image.shape[0]}; matching live intrinsics; no calibration saved.', flush=True)
                return
            # Allow exposure to settle before accumulating calibration samples.
            if time.monotonic() - start < 2:
                continue
            try:
                result = observer.estimate(image)
                ready = samples.add(result)
                shown = observer.annotate(image, result)
                message = f'Samples {len(samples.results)}/{args.samples}, stable={ready}, RMS={result["reprojection_rms_px"]:.3f}px'
            except ValueError as error:
                samples.clear()
                ready = False
                shown = image.copy()
                message = str(error)
            if time.monotonic() - last_log > 1:
                print(message, flush=True)
                last_log = time.monotonic()
            save_requested = not args.manual_save
            if args.show:
                cv2.putText(shown, message, (12, shown.shape[0]-45), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,255), 1)
                cv2.putText(shown, 's: save stable result | q: quit' if args.manual_save else 'AUTO SAVE when stable | q: quit',
                            (12, shown.shape[0]-20), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,255,255), 2)
                cv2.imshow('Orbbec table calibration', shown)
                pressed = cv2.waitKey(1) & 255
                if pressed in (ord('q'), 27):
                    raise RuntimeError('Cancelled; no calibration saved')
                save_requested = save_requested or pressed == ord('s')
            if ready and save_requested:
                doc = save_observation(out, observer, samples.results, image,
                                 camera_id=f'arx_x5_gemini335_{args.serial}',
                                 source=f'OrbbecSDK serial={args.serial}; raw color MJPG {args.width}x{args.height}@{args.fps}')
                # Table origin is directly below the printed board origin.
                # Keep X/Y along the board and Z into the table (right handed).
                table_T_board=np.eye(4)
                table_T_board[2,3]=-args.board_height_mm/1000
                table_T_camera=table_T_board@np.asarray(doc['board_T_camera'])
                write_json(out/'table_calibration.json',dict(
                    calibration_kind='camera_to_table',
                    table_T_camera=table_T_camera.tolist(),
                    camera_T_table=np.linalg.inv(table_T_camera).tolist(),
                    convention='A_T_B maps B to A; metres; column vectors',
                    table_axes='origin below board top-left outer corner; X right and Y down along printed pattern; Z into tabletop',
                    board_height_above_table_mm=args.board_height_mm,
                    assumption='Printed board is parallel to tabletop; height is user supplied, not measured by camera',
                    camera_serial=args.serial, quality=doc['quality'],
                    robot_base_T_camera=None, complete_hand_eye_calibration=False,
                    reference='moveit_calibration fbf21aa7c76258f18966fd68f555144073889caf: handeye_target_charuco.cpp',
                    detector='detectMarkers(CORNER_REFINE_NONE) -> interpolateCornersCharuco(K,D) -> estimatePoseCharucoBoard',
                    observation_file='observation.json'))
                print(f'Saved {out / "table_calibration.json"}',flush=True)
                return
        raise RuntimeError('Timed out without a stable board observation; no calibration saved')
    finally:
        if started:
            pipeline.stop()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
