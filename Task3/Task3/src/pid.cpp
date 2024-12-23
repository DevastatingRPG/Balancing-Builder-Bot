class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double dt, double integral_limit = 0, double output_limit = 0, double derivative_filter = 0.1)
        : kp(kp), ki(ki), kd(kd), dt(dt), integral_limit(integral_limit), output_limit(output_limit), derivative_filter(derivative_filter), integral(0), prev_err(0), prev_derivative(0) {}

    double compute(double err)
    {
        // Integral with anti-windup
        integral += err * dt;
        // if (integral_limit != 0)
        // {
        //     integral = std::max(std::min(integral, integral_limit), -integral_limit);
        // }

        // Derivative with filtering
        double derivative = (err - prev_err) / dt;

        // PID response
        double proportional = kp * err;
        double integral_term = ki * integral;
        double derivative_term = kd * derivative;
        double response = proportional + integral_term + derivative_term;

        // Debug prints
        // std::cout << "Proportional: " << proportional << ", Integral: " << integral_term << ", Derivative: " << derivative_term << ", Response: " << response << std::endl;

        // Limit output
        // if (output_limit != 0)
        // {
        //     response = std::max(std::min(response, output_limit), -output_limit);
        // }

        // Update previous values
        prev_err = err;

        return response;
    }

    void reset()
    {
        integral = 0;
        prev_err = 0;
    }

    void setTunings(double new_kp, double new_ki, double new_kd)
    {
        kp = new_kp;
        ki = new_ki;
        kd = new_kd;
    }

private:
    double kp;
    double ki;
    double kd;
    double dt;
    double integral;
    double prev_err;
    double integral_limit;
    double output_limit;
    double derivative_filter;
    double prev_derivative;
};