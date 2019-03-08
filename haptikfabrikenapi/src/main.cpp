#ifdef EXECUTABLE
/*
 *
 *
 *   Fs Haptic Device example integration
 *   Note: set your ethernet IP to 192.168.0.132, it will respond (with forces) to
 *         any incoming UDP messages (positions) on port 47111.
 *         mbed controller is on 192.168.0.130 (but it does not matter on this side).
 *
 *   Windows:
 *         To compile, set the folder of your Boost installation in udphaptics.pro
 *         To run, make super sure you disable windows firewall including
 *         "domain", "public" and "private" profile. The "domain" profile is only
 *         found under Windows firewall->Advanced (thank you MS).
 */
#include <iostream>
#include "fshapticdevicethread.h"
#include "fsusbhapticdevicethread.h"
#include "fsdaqhapticdevicethread.h"
#include "webserv.h"

#include <thread>
#include <chrono>
#include <sstream>


#define RENDER_BOX
#define LOOP
//#define POSITION_CONTROL
constexpr int motor=0;
constexpr double P = 0.01;
constexpr double I = 0;
constexpr double D = 0.0001;







// ******************** FOR LINUX KEYBOARD LOOP BREAK ***********
#include <stdio.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <stropts.h>

int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}
// *************************************************************

#include <deque>





















int main(int, char**)
{
    // Select model
    Kinematics::configuration c = Kinematics::configuration::polhem_v1();
    //Kinematics::configuration c = Kinematics::configuration::woodenhaptics_v2015();
    //Kinematics::configuration c = Kinematics::configuration::aluhaptics_v2();
    //Kinematics::configuration c = Kinematics::configuration::vintage();

    // Should the thread block and wait for at least one new position message before continue? (may improve stability)
    // Only implemented in UDP currently.
    bool wait_for_next_message = false;

    // Create haptics communication thread. If configuration is emitted it reads from ~/woodenhaptics.json,
    // and if that file does not exist it creates it. Only available in unix-like systems for now.
    //FsHapticDeviceThread fs(wait_for_next_message);
#ifdef USE_USB_HID
    FsUSBHapticDeviceThread fs(wait_for_next_message, c);
#elif USE_DAQ
    FsDAQHapticDeviceThread fs(wait_for_next_message, c);
#else
    FsHapticDeviceThread fs(wait_for_next_message, c);
#endif

    // Optionally set max millimaps according to your escons (default 2000). Might need to set in firmware too.
    fs.max_milliamps = 2000;

    // Open the communcication
    fs.open();

    // Verbose or not (set to at least 1 to show debug messages)
    int verbose=3;


    Webserv w;
    w.initialize();



#ifdef POSITION_CONTROL
    using namespace std::chrono;
    duration<int, std::micro> d{250000};
    std::this_thread::sleep_for(d);
    cout << "\n\nUse Q to quit. 1 to step left, 2 to step right. \nReady? Press a key and enter to start.\n";

    // Wait for new key hit
    while(!_kbhit());
    char pelle;
    cin >> pelle;


    constexpr int sleep_us = 100;
    constexpr unsigned int log_entries=10;

    int printcount=0;
    double prev_deg=0;
    int active_phase = 1;
    double goal_deg=0;

    deque<double> error_history;



    int enc[6];
    fs.getEnc(enc);
    int start_enc = enc[motor];

    while(active_phase){
        if(_kbhit()){
            cin >> pelle;

            if(pelle=='2')
                goal_deg += 5;
            if(pelle=='1')
                goal_deg -= 5;
            if(pelle=='q') {
                active_phase=0;
                continue;
            }
        }




        fs.getEnc(enc);
        constexpr double pi = 3.141592;
        double theta = pi*2.0*(enc[motor]-start_enc)/4000.0;
        //while(theta>pi*2) theta-=pi*2;
        const double deg = 360.0*theta/(pi*2.0);


        // Motor current
        double current = 0;
        {//if(deg < 360 && deg > -360){

            //double goal = (active_phase-1)*10;
            //if(active_phase == 1) goal = 20;
            double error = goal_deg-deg;


            double sum=0;
            for(auto e : error_history)
                sum+=e;

            error_history.push_back(error);
            double prev_error = goal_deg-prev_deg;
            if(error_history.size()>log_entries){
                prev_error = error_history.front();
                error_history.pop_front();
            }



            double delta_t = 0.000001*sleep_us*log_entries;
            double error_prim = (error - prev_error)/delta_t;
            double error_integrative = delta_t*(error+prev_error)/2;

            // Exact integrative
            //error_integrative = sum;

            current = P*error + I*error_integrative + D*error_prim;
            if(!(printcount%100))
                cout << "Goal: " << goal_deg << " Error: " << error << " P*: " << P*error
                     << " Ierror: " << error_integrative << " I*: " << I*error_integrative
                     << " error': " << error_prim << " D*: " << D*error_prim << " \n";

            // max 3 amps
            if(current>1.9) current=1.9;
            if(current<-1.9) current =-1.9;

        }
	if(motor==0)
	        fs.setCurrent(fsVec3d(-current,0,0));
	if(motor==1)
	        fs.setCurrent(fsVec3d(0,current,0));
	if(motor==2)
	        fs.setCurrent(fsVec3d(0,0,current));

        //cout << 10*i/3 << "\n";
        //setVolt(0.0,0); // 10 = 3A, 1=0.3A

        if(!(printcount++%100))
            cout << "Angle: " << theta << " rad  " << deg << " deg. Current: " << current << " amps\n\n";


        prev_deg = deg;

        // Sleep 1ms
        using namespace std::chrono;
        duration<int, std::micro> d{sleep_us};
        std::this_thread::sleep_for(d);
    }

    fs.setCurrent(fsVec3d(0,0,0));


    cout << "Done.\n";

    // Wait for new key hit
    while(!_kbhit());



#endif










#ifdef LOOP
    // Main loop doing some haptic rendering
    for(;;){
        fsVec3d pos = fs.getPos();
        int enc[6];
        fs.getEnc(enc);
        int ma[3];
        fs.getLatestCommandedMilliamps(ma);
        stringstream ss;

        // Uncomment this line to improve speed. Just for info.
        if(verbose){
            ss << "\"DeviceName\": \"" << fs.kinematics.m_config.name << "\",\n";
            ss << "\"Encoders\": [" << enc[0] << ", " << enc[1] << ", " << enc[2] << ", " << enc[3]
               <<", " << enc[4] << ", " << enc[5] << "],\n";
            ss << "\"CommandedMilliamps\": [" << ma[0] << ", " << ma[1] << ", " << ma[2] << "],\n";

            if(verbose>=3){
              ss << "\"Position\": [" << toString(pos) << "],\n";
              ss << "\"Orientation\": [\n" << toString(fs.getRot()) << "],\n";
              ss << "\"BodyAngles\": [" << toString(fs.getBodyAngles()) << "],\n";
    	      ss << "\"Configuration\": " << toJSON(c) << ",\n";
            }
        }

        // Haptic rendering of a surrounding box
        fsVec3d f = fsVec3d(0,0,0);
//#define RENDER_BOX
#ifdef RENDER_BOX
        double k=200;
        double b=0.03;
        double x=pos.x();
        double y=pos.y();
        double z=pos.z();
        double fx,fy,fz;
        fx=0;fy=0;fz=0;

        if(x >  b) fx = -k*(x-b);
        if(x < -b) fx = -k*(x+b);
        if(y >  b) fy = -k*(y-b);
        if(y < -b) fy = -k*(y+b);
        if(z >  b) fz = -k*(z-b);
        if(z < -b) fz = -k*(z+b);
        f = fsVec3d(fx,fy,fz);
#endif

        if(verbose){
            ss << "\"CommandedForce\":   [" << f.x() << ", " << f.y() << ", " << f.z() << "]\n";
            //ss << std::string(24-9,'\n');
            w.setMessage(ss.str());
            this_thread::sleep_for(std::chrono::microseconds(100));
        }
        //if(verbose>1) std::cout << ss.str();

        // Set force
#ifdef RENDER_BOX
        fs.setForce(f);
#endif
#ifdef POSITION_CONTROL



        fsVec3d amps = fsVec3d(2,0,0);
        fs.setCurrent(amps);
#endif
    }


#endif
}

#endif
