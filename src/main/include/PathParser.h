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
    // std::string filename =  ../src/main/deploy/pathplanner/paths/New_Path.path ;

    std::vector<frc::Translation3d> coords;

    // std::ifstream file(filename);
    
    std::string line;

    char* file_content;
    std::string file_content_s = "{version :  2025.0 , waypoints : [ { anchor : { x : 8.037102272727275, y : 5.894673295454545 }, prevControl : null, nextControl : { x : 6.991532916472671, y : 5.879126070691161 }, isLocked : false, linkedName : null }, { anchor : { x : 5.1553125, y : 5.166747159090908 }, prevControl : { x : 6.265866333176757, y : 5.786359927035319 }, nextControl : null,isLocked : false,linkedName : null}], rotationTargets : [], constraintZones : [], pointTowardsZones : [], eventMarkers : [], globalConstraints : { maxVelocity : 1.0, maxAcceleration : 1.0, maxAngularVelocity : 540.0, maxAngularAcceleration : 720.0, nominalVoltage : 12.0, unlimited : false},goalEndState : { velocity : 0, rotation : -125.1341930569156}, reversed : false, folder : null, idealStartingState : { velocity : 0, rotation : -178.60281897270355}, useDefaultConstraints : true}";
    std::string x_str;
    std::string y_str;
    std::string z_str;
    std::string r_str;

    if(coords.size() == 0)
    {
        for(int i = 0; i < file_content_s.size(), i++;)
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

            if (file_content[i] == 'r' && file_content[i+1] == 'o' && file_content[i+2] == 't' && file_content[i+10] == ':')
            {
                r_str = file_content[i+12] + file_content[i+13] +
                file_content[i+14] + file_content[i+15] + file_content[i+16] +
                file_content[i+17] + file_content[i+18];
            }

        }

        coords.push_back( frc::Translation3d ( units::meter_t ( std::stod(x_str) ), units::meter_t ( std::stod(y_str) ) , units::meter_t ( std::stod(r_str) ) ) ); // z is actually a rotaion value
    }

    return coords;

}

void OutputXYROTvals()
{   //as the name implies, it outputs the x, y, and rotational values
    //for (frc::Translation3d coord: find_values())
    //{
    if(find_values().size() > 1)
    {

        frc::SmartDashboard::PutNumber( "X val:"  , find_values()[0].X().value());
        frc::SmartDashboard::PutNumber( "Y val:"  , find_values()[0].Y().value());
        frc::SmartDashboard::PutNumber( "ROT val:"  , find_values()[0].Z().value());
        frc::SmartDashboard::PutNumber( "size:", find_values().size());
    }
    else
    {
        frc::SmartDashboard::PutNumber( "size:", find_values().size());
    }
      
    //}
}