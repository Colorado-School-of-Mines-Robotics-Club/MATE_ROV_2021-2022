from rcl_interfaces.msg import ParameterDescriptor

DEV_MODE = "dev_mode"
OUTPUT_NAME = "output_name"
CAMERA_INPUT = "camera_input"
IMAGE_DIR = "image_dir"
IMAGE_PATH = "image_path"
USE_CAMERA = "use_camera"
FRAME_RATE = "frame_rate"


def declare_params(node):
    node.declare_parameter(DEV_MODE, False, descriptor=ParameterDescriptor(description="set to dev mode"))
    node.declare_parameter(
        OUTPUT_NAME,
        "bird_image",
        descriptor=ParameterDescriptor(description="image from front downward facing camera for detecting birds"),
    )
    node.declare_parameter(
        CAMERA_INPUT,
        0,
        descriptor=ParameterDescriptor(description="input value of the camera"),
    )
    node.declare_parameter(
        IMAGE_PATH,
        "/data/birdseye/bird_finder/images/test-image.png",
        descriptor=ParameterDescriptor(description="path for single file if setup to publish one img"),
    )
    node.declare_parameter(
        USE_CAMERA,
        True,
        descriptor=ParameterDescriptor(description="use camera or if false fall back to dir of images"),
    )
    node.declare_parameter(
        FRAME_RATE,
        30,
        descriptor=ParameterDescriptor(description="rate to publish frames"),
    )
