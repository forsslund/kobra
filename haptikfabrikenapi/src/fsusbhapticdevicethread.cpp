#include "fsusbhapticdevicethread.h"
#include "../external/hidapi/hidapi.h"


// For USB HID version
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>


namespace haptikfabriken {


void FsUSBHapticDeviceThread::thread()
{
    // For debugging
    num_sent_messages = 0;
    num_received_messages = 0;

    // Set protocol 1=Old usb, 2=April 2018
    int protocol_version = 1;

    // Open connection
    devs = hid_enumerate(0x0, 0x0);
    cur_dev = devs;
    while (cur_dev) {
        printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
        printf("\n");
        printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
        printf("  Product:      %ls\n", cur_dev->product_string);
        printf("  Release:      %hx\n", cur_dev->release_number);
        printf("  Interface:    %d\n",  cur_dev->interface_number);
        printf("\n");
        cur_dev = cur_dev->next;
    }
    hid_free_enumeration(devs);

    // Open the device using the VID, PID,
    // and optionally the Serial number.
    handle = hid_open(0x1234, 0x6, NULL);
    if (!handle) {
        std::cout << "unable to open device. Is it plugged in and you run as root?\n";
        //return 1;
    }
    // Set the hid_read() function to be non-blocking.
    if(handle)
        hid_set_nonblocking(handle, 1);

    if(handle)
        std::cout << "Opened USB Connection" << std::endl;




    for(;;){
        if(!handle) continue;


        // **************** RECEIVE ***************
        int res=0;
        int count=0;
        for(int i=0;i<15;++i) buf[i]=0;
        while (res == 0) {
            res = hid_read(handle, buf, sizeof(buf));
            //if(res!=0)
            //    std::cout << res << " bytes received";
            if(res==14 || res==8) // Got a correct message
                //incoming_msg = *reinterpret_cast<woodenhaptics_message*>(buf);
                hid_to_pc = *reinterpret_cast<hid_to_pc_message*>(buf);
            usleep(5);
            count++;
            //if(!(count%1000)) std::cout << "(count: " << count << ")\n";
        }
        //std::cout << "count: " << count << "\n";

        int flush=0;
        while(int res2 = hid_read(handle, buf, sizeof(buf))){
            if(res==14 || res==8) // Got a correct message
                //incoming_msg = *reinterpret_cast<woodenhaptics_message*>(buf);
                hid_to_pc = *reinterpret_cast<hid_to_pc_message*>(buf);
            ++flush;
        }
        if(flush)
            std::cout << "Flushed " << flush << " messages." << std::endl;
        //lost_messages += flush;








        // *************** COMPUTE POSITION ***********
        // Compute position
        int ch_a=hid_to_pc.encoder_a;
        int ch_b=hid_to_pc.encoder_b;
        int ch_c=hid_to_pc.encoder_c;
        fsVec3d pos = kinematics.computePosition(ch_a,ch_b,ch_c);
        int base[] = {ch_a, ch_b, ch_c};
        int rot[]  = {hid_to_pc.encoder_d, hid_to_pc.encoder_e, hid_to_pc.encoder_f}; // Encoder d,e,f here if recevied
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
        num_received_messages += 1 + flush;
        mtx_pos.unlock();







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



    }


}

void FsUSBHapticDeviceThread::close()
{
    //close HID device
    if(handle){
        hid_close(handle);
        hid_exit();
    }
}

}
