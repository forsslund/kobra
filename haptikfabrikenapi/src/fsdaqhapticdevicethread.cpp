
#include "fsdaqhapticdevicethread.h"
#include "../external/sensoray/826api.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <ratio>


namespace haptikfabriken {


using namespace std;


// Helper functions for getPosition & setForce
//==============================================================================
int signedenc(const uint chan){
    constexpr unsigned int maxdata = 0xFFFFFFFF; // 32 bit
    uint counts;
    S826_CounterRead(0,chan,&counts);
    if(counts >= maxdata/2)
        return counts - maxdata;
    return int(counts);
}

void setVolt(double v, int motor){
    if(v > 10 || v< -10) { printf("Volt outside +/- 10 Volt\n"); return; }

    // -10V to +10V is mapped from 0x0000 to 0xFFFF
    unsigned int signal = (v+10.0)/20.0 * 0xFFFF;
    S826_DacDataWrite(0,motor,signal,0);
}



void FsDAQHapticDeviceThread::thread()
{

    // Open connection
    int status = S826_SystemOpen();

    // Initialize counters for channel 0,1,2 AND 3,4,5 (gimbal)
    for(unsigned int i=0;i<6;++i){

        constexpr uint multiples_of_20ns = 65535; // 65535 = 1.3107 ms (maximum)
        constexpr uint ENABLE_IX_FILTER = (1 << 31);
        constexpr uint ENABLE_CK_FILTER = (1 << 30);

        S826_CounterFilterWrite(0,i,0);
        //S826_CounterFilterWrite(0,i,multiples_of_20ns | ENABLE_IX_FILTER | ENABLE_CK_FILTER);
        S826_CounterFilterWrite(0,i,multiples_of_20ns | ENABLE_IX_FILTER );


        S826_CounterModeWrite(0,i,0x70);
        S826_CounterPreloadWrite(0,i,0,0); // Load 0
        S826_CounterPreload(0,i,0,0);


        // Configure snapshot to occur when IX signal either rises or falls,
        // this is to get a callback when i.e. a button is pressed that we
        // have wired to IX channel (since we are not using the encoder's IX)
        //S826_CounterSnapshotConfigWrite(0, i, S826_SSRMASK_IXRISE |
        //                                      S826_SSRMASK_IXFALL, S826_BITWRITE);

        // Enable
        S826_CounterStateWrite(0,i,1);

    }
    for(unsigned int i=0;i<3;++i){
        // And range of analoge signal to escon
        S826_DacRangeWrite(0,i,3,0); //-10 to 10 V

        // Enable ESCON driver by a digial signal, which in our case
        // is an analogue signal due to the fact that we only have the
        // analgoue and encoder breakout board of the S826.
        //
        S826_DacDataWrite(0,4+i,0xFFFF,0); // Channel 4,5,6 = Pin 41,43,45

    }

    setVolt(0,0);


    std::cout << "Opened DAQ Connection: " << status << std::endl;




    while(running){


        // **************** RECEIVE ***************
        //int chosen_motor=0;
        uint ctstamp;
        uint reason;
        uint counter;
        //const uint chan = chosen_motor;


        // Get any events occured by the switches (blocks for total 6us to read next in FIFO)

        for(int ix=0;ix<6;++ix){
            uint a = S826_CounterSnapshotRead(0,ix,&counter,&ctstamp,&reason,1); // wait 1us
            if(a != S826_ERR_NOTREADY){ // if not nothing in buffer

                std::cout << "Switch channel: " << ix;
                if(reason==8) std::cout << " ON";
                else if(reason==16) std::cout << " OFF";
                else std::cout << " ERROR ";
                std::cout << std::endl;
            }
        }


        //uint status;
        //S826_CounterStatusRead(0,0,&status);

        //uint counts;
        //S826_CounterRead(0,chan,&counts);
        //std::cout << "counts: " << counts << " chan: " << chan << "\n";











        // *************** COMPUTE POSITION ***********
        // Compute position

        int ch_a=signedenc(0);
        int ch_b=signedenc(1);
        int ch_c=signedenc(2);
        fsVec3d pos = kinematics.computePosition(ch_a,ch_b,ch_c);
        int base[] = {ch_a, ch_b, ch_c};
        int rot[]  = {signedenc(3), signedenc(4), signedenc(5)};
        fsRot r = kinematics.computeRotation(base,rot);
        fsVec3d angles = kinematics.computeBodyAngles(base);
        mtx_pos.lock();
        latestBodyAngles = angles;
        latestPos = pos;
        latestRot = r;
        latestEnc[0]=ch_a;
        latestEnc[1]=ch_b;
        latestEnc[2]=ch_c;
        latestEnc[3]=rot[0];
        latestEnc[4]=rot[1];
        latestEnc[5]=rot[2];
        num_received_messages += 1;
        mtx_pos.unlock();





/*

        // *************** GET WHAT TO SEND *****
        mtx_force.lock();
        fsVec3d f = nextForce;
        mtx_force.unlock();

        int enc[3] = {hid_to_pc.encoder_a,
                      hid_to_pc.encoder_b,
                      hid_to_pc.encoder_c};
        fsVec3d amps = kinematics.computeMotorAmps(f,enc);

        if(useCurrentDirectly){
            mtx_force.lock();
            amps = nextCurrent;
            mtx_force.unlock();
        }

        pc_to_hid.current_motor_a_mA = int(amps.x()*1000.0);
        pc_to_hid.current_motor_b_mA = int(amps.y()*1000.0);
        pc_to_hid.current_motor_c_mA = int(amps.z()*1000.0);

        // Cap at 2A since Escons 24/4 cant do more than that for 4s
        if(pc_to_hid.current_motor_a_mA >= max_milliamps) pc_to_hid.current_motor_a_mA = max_milliamps-1;
        if(pc_to_hid.current_motor_b_mA >= max_milliamps) pc_to_hid.current_motor_b_mA = max_milliamps-1;
        if(pc_to_hid.current_motor_c_mA >= max_milliamps) pc_to_hid.current_motor_c_mA = max_milliamps-1;

        if(pc_to_hid.current_motor_a_mA <= -max_milliamps) pc_to_hid.current_motor_a_mA = -max_milliamps+1;
        if(pc_to_hid.current_motor_b_mA <= -max_milliamps) pc_to_hid.current_motor_b_mA = -max_milliamps+1;
        if(pc_to_hid.current_motor_c_mA <= -max_milliamps) pc_to_hid.current_motor_c_mA = -max_milliamps+1;





        // **************** SEND ***************

        // Fake data
        //pc_to_hid.current_motor_a_mA=1234;
        //pc_to_hid.current_motor_b_mA=5678;
        //pc_to_hid.current_motor_c_mA=9012;
        pc_to_hid.force_motor_a_N=1111;
        pc_to_hid.force_motor_b_N=2222;
        pc_to_hid.force_motor_c_N=3333;
        pc_to_hid.debug=4444;


        unsigned char* msg_buf = reinterpret_cast<unsigned char*>(&pc_to_hid);


        int byte_length = 9;  // Old protocol
        if(protocol_version==2)
            byte_length = 15; // New protocol April 2018


        //Fill the report
        unsigned char out_buf[byte_length];
        out_buf[0] = 0;
        for (int i = 1; i < byte_length; i++) {
            out_buf[i] = msg_buf[i-1];
        }
        if(handle){
            //std::cout << "size: " << sizeof(out_buf) << "\n";
            int error = hid_write(handle,out_buf,sizeof(out_buf));
            if(error!=byte_length){
                std::cout << "hid_write return " << error << std::endl;
            }
        }

        mtx_pos.lock();
        latestCommandedMilliamps[0] = pc_to_hid.current_motor_a_mA;
        latestCommandedMilliamps[1] = pc_to_hid.current_motor_b_mA;
        latestCommandedMilliamps[2] = pc_to_hid.current_motor_c_mA;
        num_sent_messages++;
        mtx_pos.unlock();

        // SENT
        currentForce = f;
        */

        // // 0.01ms
        using namespace std::chrono;
        duration<int, std::micro> dd{10};
        std::this_thread::sleep_for(dd);


    }


}

void FsDAQHapticDeviceThread::close()
{
    //close
    // Disable power
    S826_DacDataWrite(0,4,0x0,0); // Channel 4 = Pin 41
    S826_DacDataWrite(0,5,0x0,0); // Channel 5 = Pin 43
    S826_DacDataWrite(0,6,0x0,0); // Channel 6 = Pin 45

    cout << "Done.\n";

    FsHapticDeviceThread::close();
}

}
