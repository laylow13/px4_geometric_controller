#ifndef FDCL_PARAM_H
#define FDCL_PARAM_H

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>      // std::setprecision

#include "Eigen/Dense"

namespace fdcl
{

/** \brief Saving and loading parameters
*
*  This class reads parameters from and to external files. This makes easier
*  to update parameters in the code, without compiling the code again when a
*  single parameter is changed. Please follow the below guidelines for making
*  the config files:\n
*  - Try to use the file extension with ".cfg", this is not required, though
*  helps to keep the consistency\n
*  - Parameters are expected to be grouped: each parameter must have a group
*  name and a parameter name \n
*  \t for example:
*  \t\t GPS:
*  \t\t  on: 0
*  \t\t  port: "/dev/ttyS0"
*  \t\t IMU:
*  \t\t  on: 1
*  - Parameters in the same group must include ONE space character at the
*  begining of the line, and NO spaces at the begining of the line for groups
*/
class param
{

public:
    std::string file_name;  /**< name of the config file where the data is
                             * is saved or read from
                             */
    std::fstream file_stream;  /**< fstream object for the config file */
    bool is_open = false;  /**< check if the file is open */

    param(void);
    param(std::string file_name);
    ~param(void);

    /** \fn void open(std::string fname)
     * Open the config files for reading/ writing
     * @param fname Path to the file
     */
    void open(std::string fname);


    /** \fn void close(void)
     * Closes the config files
     */
    void close(void);


    /** \fn void read(const std::string param_name, bool &value)
     * Reads a boolean type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name, bool &value);


    /** \fn void read(const std::string param_name, double &value)
     * Reads a double type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name, double &value);


    /** \fn void read(const std::string param_name, int &value)
     * Reads a int type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name, int &value);


    /** \fn void read(const std::string param_name, std::string &value)
     * Reads a string type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name, std::string &value);


    /** \fn void read(const std::string param_name, Matrix3 &value)
     * Reads a Matrix3 type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name,
            Eigen::Matrix<double, 3, 3> &value);


    /** \fn void read(const std::string param_name, Vector4 &value)
     * Reads a Vector4 type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name,
            Eigen::Matrix<double, 4, 1> &value);


    /** \fn void read(const std::string param_name, Vector3 &value)
     * Reads a Vector3 type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name,
            Eigen::Matrix<double, 3, 1> &value);

    /** \fn void read(const std::string param_name, Matrix15 &value)
     * Reads a Matrix15 type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name,
            Eigen::Matrix<double, 15, 15> &value);

    /** \fn void read(const std::string param_name, Vector2 &value)
     * Reads a Vector2 type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void read(const std::string param_name,
            Eigen::Matrix<double, 2, 1> &value);


    /** \fn void read(const std::string param_name,
     *  Eigen::MatrixBase<Derived> &value)
     *
     * Reads a any Eigen vector type parameter from the config file
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    template<typename Derived>
    void read(const std::string param_name, Eigen::MatrixBase<Derived> &value);


    /** void save(const std::string param_name, bool value)
     * Saves a given bool type parameter to the config files
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void save(const std::string param_name, bool value);


    /** void save(const std::string param_name, double value)
     * Saves a given double type parameter to the config files
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void save(const std::string param_name, double value);


    /** void save(const std::string param_name, int value)
     * Saves a given int type parameter to the config files
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void save(const std::string param_name, int value);


    /** void save(const std::string param_name, std::string value)
     * Saves a given string type parameter to the config files
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void save(const std::string param_name, const std::string value);


    /** void save(const std::string param_name, Matrix3 value)
     * Saves a given Matrix3 type parameter to the config files
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    void save(const std::string param_name,
            Eigen::Matrix<double, 3, 3>  &value);


    /** void save(const std::string param_name,
     * Eigen::MatrixBase<Derived> value)
     *
     * Saves a given Eigen type parameter to the config files
     * @param param_name name of the parameter
     * @param value      varaiable to save the read parameter value
     */
    template<typename Derived>
    void save(const std::string param_name, Eigen::MatrixBase<Derived> &value);


private:
    /** \fn std::string find_line(const std::string param_name)
     * Find the line which includes a given parameter
     * @param  std::string name if the parametr needs to be found
     * @return std::string line containing the parameter
     */
    std::string find_line(const std::string param_name);


    /**
     * Replaces a parameter value in the config file with a new values
     * @param param_name current parameter value
     * @param new_value  new parameter value
     */
    void replace_value(const std::string param_name, std::string new_value);

};  // end of fdcl::param class
}  // end of fdcl namespace

#endif
