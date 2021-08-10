#pragma once
#include "ros/ros.h"
#include <serial/serial.h>
#include "pgv100/pgv100_msg.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <cstdint>
#define debug

using namespace std;

void Error_Handler(string error_result)
{
    ROS_ERROR_STREAM(error_result);
}
class pgv_100
{
    private:
        serial::Serial PGV100_Serial;
        pgv100::pgv100_msg PGV100_Line;
        pgv100::pgv100_msg PGV100_QRCode;
        int Address;
    public:
        // Y81,115200
        pgv_100(std::string comport)
        {
            // Open pgv100
            this->pgv_open(comport);
            // Address
            this->Address = 0;
        }
        void pgv_open(std::string comport)
        {
            try
            {
                // Open Comport
                PGV100_Serial.setPort(comport);
                PGV100_Serial.setBaudrate(115200);
                PGV100_Serial.setParity(serial::parity_even);
                serial::Timeout Lto = serial::Timeout::simpleTimeout(0);
                PGV100_Serial.setTimeout(Lto);
                PGV100_Serial.open();
            }
            // Check Open or not
            catch (serial::IOException& e)
            {
                // cout << "error" << endl;
                Error_Handler("Unable to open PGV port");
            }
        } 
        // Send Request
        void Send_Request(unsigned char req)
        {
            req += this->Address;
            //
            vector<unsigned char> req_vector;
            //
            req_vector.clear();
            req_vector.push_back(req);
            PGV100_Serial.write(req_vector);
            //
            req_vector.clear();
            req_vector.push_back(255 - req);
            PGV100_Serial.write(req_vector);
        }
        //
        void Read_Data(std::vector<unsigned char>* return_buffer, int byte_size)
        {
            // Clear buffer
            return_buffer->clear();
            // Receive flag
            bool Receive_flag = false;
            while (return_buffer->size() < byte_size)
            {
                if(this->PGV100_Serial.available() > 0)
                {
                    unsigned char Receive_buffer = 0xFF;
                    // Read data from pgv100
                    this->PGV100_Serial.read( &Receive_buffer,1);
                    return_buffer->push_back(Receive_buffer);
                }
            }
            //this->PGV100_Serial.flushInput();
        }
        // BGR (0,1,2), return 0:success, 1:fail
        int Set_Color_Mode(int color)
        {
            unsigned char req_byte;
            if(color != 0)
            {
                req_byte = 0x80;
            }
            else
            {
                req_byte = 0xC0;
            }
            vector<unsigned char> req_vector;
            req_vector.clear();
            // Calculation Request Bytes
            req_byte |= (1 << (color + 2));
            //this->Send_Request(req_byte);
            // Send Color Mode Request
            this->Send_Request(req_byte);
            // Get Response
            std::vector<unsigned char> Response;
            Response.clear();
            this->Read_Data(&Response,2);
            // Check Setting Success
            if( int(Response[0]) == pow(2,color) && int(Response[0]) == pow(2,color))
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        //direction (0,1,2 ; right,left,ahead)
        void Set_Direction_Mode(int direction)
        {
            unsigned char req_byte = 0xE0;
            // set direct
            if(direction != 2)
            {
                req_byte |= ( 1 << ( direction + 2) );
            }
            else
            {
                req_byte = 0xEC;
            }
            // Send Direction Request
            this->Send_Request(req_byte);
        }
        /*************Get Response*******************/
        int Update(std::vector<unsigned char> *Response)
        {
            // Send Request
            this->Send_Request(0xC8);
            if(this->PGV100_Serial.available() < 21)return -1;
            Response->clear();
            this->PGV100_Serial.flushInput();
            this->Read_Data(Response,21);
            // this->PGV100_Serial.flushInput();
            // Get Response
            // Response->clear();
            // this->Send_Request(0xC8);
            return 0;
        }
        // pgv100_msg to pose
        geometry_msgs::Pose pgv100_msg_to_pose(pgv100::pgv100_msg *PGV_Info)
        {
            geometry_msgs::Pose pgv_pose;
            pgv_pose.position.x = PGV_Info->Position.x;
            pgv_pose.position.y = PGV_Info->Position.y;
            double angle = 0;
            
            if(PGV_Info->Angle.data > 180)angle = (PGV_Info->Angle.data) - 360;
            else angle = PGV_Info->Angle.data;
            
            // angle = PGV_Info->Angle.data;
            pgv_pose.orientation = tf::createQuaternionMsgFromYaw(angle * M_PI / 180.0);
            return pgv_pose;
        }
        // Get Mode
        void Auto_Detect_Update(pgv100::pgv100_msg *PGV_Info)
        {
            // Response
            std::vector<unsigned char> Response;
            //
            if(this->Update(&Response) == -1)
            {
                return;
            }
            // Position
            unsigned char XPS = 0;
            char YPS = 0;
            int Angle = 0;
            unsigned int Tag_Number = 0;
            // Spec (NL, 1:No Line,0:Line)
            // QRCode
            if(Response[1] & 0x40 > 0x00)
            {
                XPS = ( (Response[2] << 21 & 0x07) | Response[3] << 14 | Response[4] << 7 | Response[5]);
                YPS = (Response[6] << 7 | Response[7]);
                Angle = (Response[10] << 7 | Response[11]);
                Tag_Number = (Response[14] << 21 | Response[15] << 14 | Response[16] << 7 | Response[17]);
                // ROS_INFO("%x,%x,%x,%x",Response[14],Response[15],Response[16],Response[17]);
                PGV_Info->Mode.data = 1;
            }
            else
            //Line
            {
                // XP
                XPS = Response[2] & 0x07 << 21 | Response[3] << 14 | Response[4] << 7 | Response[5];
                // YPS
                YPS = Response[6] << 7 | Response[7];
                // Angle
                Angle = Response[10] << 7 | Response[11];
                //
                Tag_Number = (Response[14] << 21 | Response[15] << 14 | Response[16] << 7 | Response[17]);
                //
                PGV_Info->Mode.data = 0;
            }
            // Assign
            PGV_Info->Position.x = float((int8_t)XPS);
            PGV_Info->Position.y = float((int8_t)YPS);
            PGV_Info->Angle.data = int(Angle);
            PGV_Info->Tag_Number.data = int(Tag_Number);
        }
        /*
        void Line_Update()
        {
            //
            std::vector<unsigned char> Response;
            if(this->Update(&Response) == -1)return;
            // XP
            unsigned int XP = Response[2] & 0x07 << 21 | Response[3] << 14 | Response[4] << 7 | Response[5];
            // YPS
            char YPS = Response[6] << 7 | Response[7];
            // Angle
            int Angle = Response[10] << 7 | Response[11];
            //cout << (int)XP << "," << (int)YPS << "," << (int)Angle << endl;
            // return information
            PGV100_Line.Mode.data = 0;
            PGV100_Line.Position.x = float(XP);
            PGV100_Line.Position.y = float(YPS);
            PGV100_Line.Angle.data = int(Angle);
        }
        //
        void QR_Code_Update()
        {
            std::vector<unsigned char> Response;
            if(this->Update(&Response) == -1)
            {
                return;
            }
            unsigned int XPS = ( (Response[2] << 21 & 0x07) | Response[3] << 14 | Response[4] << 7 | Response[5]);
            char YPS = (Response[6] << 7 | Response[7]);
            int Angle = (Response[10] << 7 | Response[11]);
            unsigned int Tag_Number = (Response[14] << 21 | Response[15] << 14 | Response[16] << 7 | Response[17]);
            //cout << "X_Position: " << (int)XPS << "; " << "Y_Position: " <<(int)YPS << "; " << "Angle: " << (int)Angle << "; " << "Tag Number: " << int(Tag_Number) << endl;
            // return information
            PGV100_QRCode.Mode.data = 1;
            PGV100_QRCode.Position.x = float(XPS);
            PGV100_QRCode.Position.y = float(YPS);
            PGV100_QRCode.Angle.data = int(Angle);
            PGV100_QRCode.Tag_Number.data = int(Tag_Number);
        }
        */
        //
        void QR_Code_Update(pgv100::pgv100_msg *QR_Code_Info)
        {
            std::vector<unsigned char> Response;
            //
            if(this->Update(&Response) == -1)
            {
                ROS_INFO_STREAM("Not Response");
                return;
            }
            ROS_INFO_STREAM("Response");
            /*
            // Check Response
            if (! (Response[0] ^ 0x02) )
            {   
                cout << "No X" << endl;
                return;
            }
            if (! (Response[1] ^ 0x04) )
            {
                cout << "No Line" << endl;
                return;
            }
            */
            unsigned char XPS = ( (Response[2] << 21 & 0x07) | Response[3] << 14 | Response[4] << 7 | Response[5]);
            char YPS = (Response[6] << 7 | Response[7]);
            int Angle = (Response[10] << 7 | Response[11]);
            unsigned int Tag_Number = (Response[14] << 21 | Response[15] << 14 | Response[16] << 7 | Response[17]);
            //cout << "X_Position: " << (int)XPS << "; " << "Y_Position: " <<(int)YPS << "; " << "Angle: " << (int)Angle << "; " << "Tag Number: " << int(Tag_Number) << endl;
            // return information
            QR_Code_Info->Mode.data = 1;
            QR_Code_Info->Position.x = float(XPS);
            QR_Code_Info->Position.y = float(YPS);
            QR_Code_Info->Angle.data = int(Angle);
            QR_Code_Info->Tag_Number.data = int(Tag_Number);
        }
        void Line_Update(pgv100::pgv100_msg *Line_Info)
        {
            //
            std::vector<unsigned char> Response;
            if(this->Update(&Response) == -1)return;
            /*
            // Check Response
            if (! (Response[0] ^ 0x02) )
            {   
                cout << "No X" << endl;
                return;
            }
            if (! (Response[1] ^ 0x04) )
            {
                cout << "No Line" << endl;
                return;
            }
            */
            // XP
            unsigned int XP = ((Response[2] & 0x07) << 21) | (Response[3] << 14) | (Response[4] << 7) | Response[5];
            // YPS
            char YPS = Response[6] << 7 | Response[7];
            // Angle
            int Angle = Response[10] << 7 | Response[11];
            //cout << (int)XP << "," << (int)YPS << "," << (int)Angle << endl;
            // return information
            Line_Info->Mode.data = 0;
            Line_Info->Position.x = float(XP);
            Line_Info->Position.y = float(YPS);
            Line_Info->Angle.data = int(Angle);
        }
};