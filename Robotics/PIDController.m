% Trong lớp PIDController:
classdef PIDController
    properties
        Kp
        Ki
        Kd
        prev_error
        integral
        dt % Thêm một thuộc tính cho delta_t
    end
    
    methods
        function obj = PIDController(Kp, Ki, Kd, dt)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.prev_error = 0;
            obj.integral = 0;
            obj.dt = dt; % Lưu trữ delta_t
        end
        
        function output = compute(obj, error)
            % Tính toán đầu ra của bộ điều khiển PID dựa trên lỗi hiện tại
            % và thời gian lấy mẫu

            % Tính toán thành phần Proportional (P)
            P = obj.Kp * error;

            % Tính toán thành phần Integral (I)
            obj.integral = obj.integral + error * obj.dt;
            I = obj.Ki * obj.integral;

            % Tính toán thành phần Derivative (D)
            derivative = (error - obj.prev_error) / obj.dt;
            D = obj.Kd * derivative;

            % Tổng hợp các thành phần để có đầu ra của bộ điều khiển PID
            output = P + I + D;

            % Cập nhật lỗi trước đó cho lần lặp kế tiếp
            obj.prev_error = error;
        end
    end
end
