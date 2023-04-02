#include "fdcl/param.hpp"


fdcl::param::param(void)
{
    // If a file name is not defined, use 'fdcl.cfg' as the file name
    // TODO: handle errors if the file is not found
    file_name = "fdcl.cfg";
}


fdcl::param::~param(void)
{
    if (is_open) fdcl::param::close();
}


fdcl::param::param(std::string file_name)
{
    file_stream.open(file_name.c_str(), std::ios::in);
}


void fdcl::param::open(std::string fname)
{
    std::cout << "PARAM: opening " << fname << " .." << std::endl;

    file_name = fname;
    file_stream.open(file_name.c_str(), std::ios::in);
    is_open = true;
}


void fdcl::param::close(void)
{
    std::cout << "PARAM: closing " << file_name << " .." << std::endl;
    file_stream.close();
    is_open = false;
}


std::string fdcl::param::find_line(const std::string param_name)
{
    bool found_var = false;
    std::string line, group, var;
    std::string::size_type pos;

    file_stream.clear();
    file_stream.seekg(0, file_stream.beg);

    // Keep checking until the variable is found
    while(!found_var)
    {
        if(std::getline(file_stream,line) && line.length() > 0)
        {
            // If the the first character of a line is not a space,
            // that means its a name of a 'Group'.
            if(!isspace(line[0]))
            {
                // A group name ends with a colon (:), remove that and save the
                // group name to the 'group' variable
                group = line.erase(line.size() - 1);
            }

            // If a group name is a space, that means it is a variable defined
            // under some group. A parameter has the format of
            // "parameter_name: parameter_value".
            else
            {
                pos = line.find(':');
                var = line.substr(1, pos - 1);

                if(group + "." + var == param_name)
                {
                    found_var = true;
                    line = line.substr(pos + 1, line.length() - pos + 1);
                }
            }
        }
        else
        {
            std::cerr << "PARAM: ERROR: there is no entry of "
                      << param_name
                      << std::endl;
            break;
        }
    }

    return line;
}


void fdcl::param::replace_value(const std::string param_name,
        std::string new_value)
{
    std::string line, group, param, var;
    std::string::size_type pos;
    std::stringstream buf;
    std::ofstream new_file;

    file_stream.clear();
    file_stream.seekg(0,file_stream.beg);

    // Keep checking until the variable is found
    while(std::getline(file_stream, line) && line.length() > 0)
    {
        // If the the first character of a line is not a space,
        // that means its a name of a 'Group'.
        if(!isspace(line[0]))
        {
            group = line;
            group = group.erase(group.size() - 1);  // remove the last ":"
            buf << line << std::endl;
        }

        // If a group name is a space, that means it is a variable defined
        // under some group. A parameter has the format of
        // "parameter_name: parameter_value".
        else
        {
            // line of variables
            pos = line.find(':');
            var = line.substr(1, pos - 1);

            if(group + "." + var == param_name)
            {
                // line of param to replace
                buf << " " << var << ": " << new_value << std::endl;
            }
            else
            {
                // line of other parms
                buf << line << std::endl;
            }
        }
    }

    file_stream.close();

    // reopen the file in the replace mode and save the buffer to the file
    new_file.open(file_name.c_str(), std::ios::trunc);
    new_file << buf.str();
    new_file.close();

    // reopen the file in the read mode
    file_stream.open(file_name.c_str());
}


void fdcl::param::read(const std::string param_name, bool &value)
{
    int i;
    i = std::stoi(find_line(param_name));

    if (i == 0) value = false;
    else if (i == 1) value = true;
    else std::cerr << "PARAM: ERROR: bool should be either 0 or 1"
                   << std::endl;
}


void fdcl::param::read(const std::string param_name, double &value)
{
    value = std::stod(find_line(param_name));
}


void fdcl::param::read(const std::string param_name, int &value)
{
    value = std::stoi(find_line(param_name));
}


void fdcl::param::read(const std::string param_name, std::string &value)
{
    std::string line;
    std::string::size_type pos;

    line = find_line(param_name);

    pos = line.find('"');
    line.erase(0, pos + 1);  // erase the first "
    pos = line.find('"');
    line.erase(pos);  // erase the last ""
    value = line;
}


void fdcl::param::read(const std::string param_name,
        Eigen::Matrix<double, 3, 3> &M)
{
    int i, j;
    std::string line;
    std::string::size_type pos_delimiter;

    line = find_line(param_name);
    for (i = 0; i < M.rows(); i++)
    {
        for(j = 0; j < M.cols(); j++)
        {
            if((pos_delimiter = line.find(", ")) != std::string::npos)
            {
                M(i, j) = stod(line.substr(0, pos_delimiter));
                line.erase(0, pos_delimiter + 1);
            }
            else
            {
                M(i, j) = stod(line);
            }
        }
    }
}

void fdcl::param::read(const std::string param_name,
        Eigen::Matrix<double, 15, 15> &M)
{
    int i, j;
    std::string line;
    std::string::size_type pos_delimiter;

    line = find_line(param_name);
    for (i = 0; i < M.rows(); i++)
    {
        for(j = 0; j < M.cols(); j++)
        {
            if((pos_delimiter = line.find(", ")) != std::string::npos)
            {
                M(i, j) = stod(line.substr(0, pos_delimiter));
                line.erase(0, pos_delimiter + 1);
            }
            else
            {
                M(i, j) = stod(line);
            }
        }
    }
}


void fdcl::param::read(const std::string param_name,
        Eigen::Matrix<double, 4, 1> &M)
{
    int i, j;
    std::string line;
    std::string::size_type pos_delimiter;

    line = find_line(param_name);
    for (i = 0; i < M.rows(); i++)
    {
        for(j = 0; j < M.cols(); j++)
        {
            if((pos_delimiter = line.find(", ")) != std::string::npos)
            {
                M(i, j) = stod(line.substr(0, pos_delimiter));
                line.erase(0, pos_delimiter + 1);
            }
            else
            {
                M(i, j) = stod(line);
            }
        }
    }
}


void fdcl::param::read(const std::string param_name,
       Eigen::Matrix<double, 3, 1> &M)
{
    int i, j;
    std::string line;
    std::string::size_type pos_delimiter;

    line = find_line(param_name);
    for (i = 0; i < M.rows(); i++)
    {
        for(j = 0; j < M.cols(); j++)
        {
            if((pos_delimiter = line.find(", ")) != std::string::npos)
            {
                M(i, j) = stod(line.substr(0, pos_delimiter));
                line.erase(0, pos_delimiter + 1);
            }
            else
            {
                M(i, j) = stod(line);
            }
        }
    }
}


void fdcl::param::read(const std::string param_name,
       Eigen::Matrix<double, 2, 1> &M)
{
    int i, j;
    std::string line;
    std::string::size_type pos_delimiter;

    line = find_line(param_name);
    for (i = 0; i < M.rows(); i++)
    {
        for(j = 0; j < M.cols(); j++)
        {
            if((pos_delimiter = line.find(", ")) != std::string::npos)
            {
                M(i, j) = stod(line.substr(0, pos_delimiter));
                line.erase(0, pos_delimiter + 1);
            }
            else
            {
                M(i, j) = stod(line);
            }
        }
    }
}


template<typename Derived>
void fdcl::param::read(const std::string param_name,
        Eigen::MatrixBase<Derived>& M)
{
    int i, j;
    std::string line;
    std::string::size_type pos_delimiter;

    line = find_line(param_name);
    for (i = 0; i < M.rows(); i++)
    {
        for(j = 0; j < M.cols(); j++)
        {
            if((pos_delimiter = line.find(", ")) != std::string::npos)
            {
                M(i, j) = stod(line.substr(0, pos_delimiter));
                line.erase(0, pos_delimiter + 1);
            }
            else
            {
                M(i, j) = stod(line);
            }
        }
    }
}


void fdcl::param::save(const std::string param_name, bool value)
{
    int i;

    if (value == false) i = 0;
    else i = 1;

    replace_value(param_name, std::to_string(i));
}


void fdcl::param::save(const std::string param_name, double value)
{
    replace_value(param_name, std::to_string(value));
}


void fdcl::param::save(const std::string param_name, int value)
{
    replace_value(param_name, std::to_string(value));
}


void fdcl::param::save(const std::string param_name, const std::string value)
{
    std::stringstream new_value;

    new_value << "\"" << value << "\"";
    replace_value(param_name, new_value.str());
}




void fdcl::param::save(const std::string param_name,
        Eigen::Matrix<double, 3, 3> &M)
{
    int i, j;
    std::stringstream new_value;

    new_value << std::scientific;
    new_value << std::setprecision(10);

    for (i = 0; i < M.rows(); i++)
    {
        for(j = 0; j < M.cols(); j++)
        {
            new_value << M(i, j) << ", ";
        }
    }

    replace_value(param_name, new_value.str());
}


template<typename Derived>
void fdcl::param::save(const std::string param_name,
    Eigen::MatrixBase<Derived>& M
)
{
    int i, j;
    std::stringstream new_value;

    new_value << std::scientific;
    new_value << std::setprecision(10);

    for (i = 0; i < M.rows(); i++)
    {
        for(j = 0; j < M.cols(); j++)
        {
            new_value << M(i, j) << ", ";
        }
    }

    replace_value(param_name, new_value.str());
}
