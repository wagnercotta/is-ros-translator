class GatewayError(Exception):
    def __init__(self, message: str, code: str = "gateway_error", status: int = 400):
        super().__init__(message)
        self.code = code
        self.status = status


class AccessDenied(GatewayError):
    def __init__(self, message: str):
        super().__init__(message, "forbidden", 403)


class NoRobot(GatewayError):
    def __init__(self, robot_id: str):
        super().__init__(f"Unknown robot: {robot_id}", "robot_not_found", 404)


class ROSUnavailable(GatewayError):
    def __init__(self, message: str):
        super().__init__(message, "ros_unavailable", 503)


class InvalidRequest(GatewayError):
    def __init__(self, message: str):
        super().__init__(message, "invalid_request", 422)


class OperationTimeout(GatewayError):
    def __init__(self, message: str):
        super().__init__(message, "timeout", 504)
