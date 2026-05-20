import logging


def setup_logger(node, log_level):
    """
    Set up the logger with appropriate log level and formatting.

    Args:
        node: ROS2 node instance for logging through ROS system
        log_level: The log level from config (DEBUG, INFO, WARNING, ERROR, CRITICAL)

    Returns:
        Configured logger object
    """
    # Convert string log level to logging constants
    level_map = {
        'DEBUG': logging.DEBUG,
        'INFO': logging.INFO,
        'WARNING': logging.WARNING,
        'ERROR': logging.ERROR,
        'CRITICAL': logging.CRITICAL
    }

    # Default to INFO if level not recognized
    numeric_level = level_map.get(log_level, logging.INFO)

    # Configure root logger
    logger = logging.getLogger()
    logger.setLevel(numeric_level)

    # Remove any existing handlers to avoid duplicates
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)

    # Console handler with improved formatting
    console = logging.StreamHandler()
    console.setLevel(numeric_level)

    # Format: timestamp - level - component - message
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(name)s - %(message)s')
    console.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(console)

    # Map Python logging levels to ROS2 logging levels
    def log_bridge(record):
        if record.levelno >= logging.CRITICAL:
            node.get_logger().fatal(record.getMessage())
        elif record.levelno >= logging.ERROR:
            node.get_logger().error(record.getMessage())
        elif record.levelno >= logging.WARNING:
            node.get_logger().warn(record.getMessage())
        elif record.levelno >= logging.INFO:
            node.get_logger().info(record.getMessage())
        elif record.levelno >= logging.DEBUG:
            node.get_logger().debug(record.getMessage())

    # Create a handler that bridges Python logging to ROS2 logging
    class ROS2LogHandler(logging.Handler):
        def emit(self, record):
            log_bridge(record)

    # Add ROS2 log handler
    ros2_handler = ROS2LogHandler()
    ros2_handler.setLevel(numeric_level)
    logger.addHandler(ros2_handler)

    return logger