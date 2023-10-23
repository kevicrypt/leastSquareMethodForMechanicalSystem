#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include "Eigen/Dense"
#include<iostream>

using namespace Eigen;

// This function is from this source https://www.gormanalysis.com/blog/reading-and-writing-csv-files-with-cpp/
std::vector<std::pair<std::string, std::vector<double>>> read_csv(std::string filename){
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<double>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    double val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while(std::getline(ss, colname, ',')){
            
            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, std::vector<double> {}});
        }
    }

    // Read data, line by line
    while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);
        
        // Keep track of the current column index
        int colIdx = 0;
        
        // Extract each integer
        while(ss >> val){
            
            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);
            
            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();
            
            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}

int main() {
    // Read motordata from file
    std::vector<std::pair<std::string, std::vector<double>>> motorData = read_csv("motordata.csv");

    std::vector<double> time = motorData.at(0).second;
    // Map our time vector to an algebraic vector
    Map<RowVectorXd> timeVector (time.data(),time.size());

    std::vector<double> motorTorque = motorData.at(1).second;
    // Map our torque vector to an algebraic vector
    Map<RowVectorXd> motorTorqueVector (motorTorque.data(),motorTorque.size());

    std::vector<double> motorAcceleration = motorData.at(2).second;
    // Map our acceleration vector to an algebraic vector
    Map<RowVectorXd> motorAccelerationVector (motorAcceleration.data(),motorAcceleration.size());

    std::vector<double> motorVelocity = motorData.at(3).second;
    // Map our velocity vector to an algebraic vector
    Map<RowVectorXd> motorVelocityVector (motorVelocity.data(),motorVelocity.size());

    std::vector<double> motorPosition = motorData.at(4).second;
    // Map our position vector to an algebraic vector
    Map<RowVectorXd> motorPositionVector (motorPosition.data(),motorPosition.size());

    // Create data vector (psi)
    MatrixXd psi(motorVelocityVector.cols(), motorVelocityVector.rows() + motorAccelerationVector.rows() + motorTorqueVector.rows());
    psi << -1 * motorVelocityVector.transpose(), -1 * motorAccelerationVector.transpose(), motorTorqueVector.transpose();

    // Create parameter vector (theta) 
    MatrixXd theta(1,3);
    theta = ((psi.transpose() * psi).inverse()) * psi.transpose() * motorPositionVector.transpose();
    
    // Transfer our algebraic theta vector to a c++ vector
    std::vector<double> thetaVector (theta.data(), theta.data() + theta.size());

    // Assigning parameters of the system
    int m = 7; // pendulum mass
    float l = 0.24; // pendulum length
    float g = 9.81; // gravity acceleration
    double i = (m * g * l) * thetaVector[2]; // gear ratio
    double J = (m * g * l) * thetaVector[1] / (i*i); // moment of inertia
    double B = (m * g * l) * thetaVector[0] / (i*i); // friction coefficient

    std::cout << "Results for mass = " << m << "kg, length = " << l << "cm" << std::endl; 
    std::cout << "Gear ratio i: " << i ; 
    std::cout << "\nMoment of inertia J: " << J << "\nFriction coefficient B: " << B << std::endl ; 

    return 0;
}
