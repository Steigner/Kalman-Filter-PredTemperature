#include "eigen/Eigen/Dense"
#include <iostream>
#include <string>

class Kalman_Temp_Pred {
    private:
        // eigen matrix of measurements
        Eigen::MatrixXd eigen_measurements;
        // init kovariance matrix of error
        Eigen::Matrix2d P;
        // state transition matrix
        Eigen::Matrix2d F;
        // kovariance matrix of error
        Eigen::Matrix2d Q;
        // vector of measurements - temperature
        Eigen::Vector2d H;
        // matrix of measurements - temperature
        Eigen::MatrixXd R;
        // vecotr of init state
        Eigen::Vector2d x;
        
    public:
        // matrix of measurements
        std::vector<std::vector<double>> measurements;
        
    void init(){
        // init matrix
        // initial uncertainty / error
        P = Eigen::Matrix2d::Identity() * 10;
        // temperature changes
        F = Eigen::Matrix2d::Identity();
        // deviations in prediction
        Q = Eigen::Matrix2d::Identity() * 0.1;
        R = Eigen::MatrixXd::Ones(1, 1); 

        H << 1,0;
        F(0,1) = 1;
        Q(0,1) = 0.1;
        // init state vector [T, T_dot]
        x << 20, 0;

        // transfer matrix of measurements to eigen matrix
        int rows = measurements.size();

        if(rows == 0){
            throw std::runtime_error("Zero size of input measurements");
        };

        int col_len = measurements[0].size();

        for(int row = 0; row < rows; row++){
            if(col_len != measurements[row].size()){
                throw std::runtime_error("Different size of columns");
            }
        };

        // if rows > 0 than measurements[0].size() if not 0
        int cols = rows > 0 ? measurements[0].size() : 0;
        
        eigen_measurements.resize(rows, cols);
        
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                eigen_measurements(i, j) = measurements[i][j];
            }
        };

        // print transformed measurements 
        // std::cout << eigen_measurements << std::endl;
    }

    // call computation of kalman filter
    double kalman_filter(){
        for(int i=0; i < eigen_measurements.rows(); i++ ){
            auto [x, P] = update(i);
        }

        auto [x, P] = update(0, true);
        std::cout << "Excpeted temp °C: " << x(0) << std::endl;

        double x_temp_pred = x(0);
        return x_temp_pred;
    }

    // update method 
    std::tuple<Eigen::Vector2d, Eigen::Matrix2d> update(const int i, bool pred = false){
        // prediction
        x = F * x;
        P = ((F * P) * F.transpose()) + Q;

        // update
        if (pred == false){
            // innovation measuring residue
            Eigen::MatrixXd y = eigen_measurements.row(i) - (H.transpose() * x);
            // inovation kovariance
            Eigen::MatrixXd S = (H.transpose() * (P * H)) + R;
            // kalman inovation
            Eigen::MatrixXd K = (P * H) * S.inverse();

            // updated status vector
            x = x + (K * y); 
            P = P - ((K * H.transpose()) * P);
        }

        return std::make_tuple(x, P);
    };

    void print_inputs(){
       std::cout << "[INFO] Input measurements °C :\n" << eigen_measurements << "\n" << std::endl;
    };
};

int main() {
    try{
        Kalman_Temp_Pred kalman;
        // simulated temperature
        kalman.measurements = {{20}, {21}, {21.5}, {22}};
        kalman.init();
        kalman.print_inputs();
        kalman.kalman_filter();

    } catch (const std::runtime_error& e) {
        std::cerr << "Caught runtime_error: " << e.what() << std::endl;
    }

    return 0;
}