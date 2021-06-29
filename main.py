import rclpy
import argparse
from src.lane_tracker import LaneTracking

def main(args):
    rclpy.init()
    tracker = LaneTracking(args.model_path)
    rclpy.spin(tracker)

    tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', '-p', required=True,
                        help="path to tflite model", type=str)
    args = parser.parse_args()
    main(args)
