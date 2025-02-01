#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>

/*   MAKE COMMENTS FOR CODE BLOCKS   */

std::vector<frc::Translation3d> find_values() 
{
    std::string filename = "C:/src/2025-Reefscape/src/main/deploy/pathplanner/Tests.path";

    std::vector<frc::Translation3d> coords;

    std::ifstream file(filename);

    std::string line;

    std::string file_content = "";

    std::string x_str;
    std::string y_str;
    std::string z_str;
    std::string r_str;


    if(file.is_open())
    {
        while(std::getline(file, line))
        {
            file_content += line + "\n";
        }
        file.close();
    }
    else
    {
        std::cout << "Unable to open file.\n";
    }

    if(coords.size() > 1)
    {
        for(int i = 0; i < coords.size(), i++;)
        {
            if (file_content[i] == 'x' && file_content[i+2] == ':')
            {
                x_str = file_content[i+4] + file_content[i+5] +
                file_content[i+6] + file_content[i+7] + file_content[i+8] + 
                file_content[i+9];
            }

            if (file_content[i] == 'y' && file_content[i+2] == ':')
            {
                y_str = file_content[i+4] + file_content[i+5] +
                file_content[i+6] + file_content[i+7] + file_content[i+8] +
                file_content[i+9];
            }

            if (file_content[i] == 'h' && file_content[i+1] == 'o' && file_content[i+2] == 'l' && file_content[i+13] == ':')
            {
                r_str = file_content[i+17] + file_content[i+18] +
                file_content[i+19] + file_content[i+20] + file_content[i+21] +
                file_content[i+22] + file_content[i+23];
            }

        }

        coords.push_back( frc::Translation3d ( units::meter_t ( std::stod(x_str) ), units::meter_t ( std::stod(y_str) ) , units::meter_t ( std::stod(r_str) ) ) ); // z is actually a rotaion value
    }

    return coords;

}

void OutputXYROTvals()
{
    for (frc::Translation3d coord: find_values())
    {
        frc::SmartDashboard::PutNumber("X val: ", coord.X().value());
        frc::SmartDashboard::PutNumber("Y val: ", coord.Y().value());
        frc::SmartDashboard::PutNumber("ROT val: ", coord.Z().value());
      
    }
}