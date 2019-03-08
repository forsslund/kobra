#ifndef HAPTIKFABRIKENAPI_H
#define HAPTIKFABRIKENAPI_H

/*
 *  Haptikfabriken API
 *  Methods to interact with our haptic devices
 *  e.g. Polhem and WoodenHaptics.
 *
 *  This library is licenced under LGPL (attribution and share-alike)
 *  This header file is licenced under BSD (do what you want)
 *  (C) Forsslund Systems AB 2019
 *
 */
#include <string>

namespace haptikfabriken {

struct fsVec3d {
    double m_x,m_y,m_z;
    fsVec3d(double x, double y, double z):m_x(x),m_y(y),m_z(z){}
    fsVec3d():m_x(0),m_y(0),m_z(0){}
    double x(){ return m_x; }
    double y(){ return m_y; }
    double z(){ return m_z; }
    void zero(){m_x=0;m_y=0;m_z=0;}
};
inline fsVec3d operator*(const double& d, const fsVec3d& v){ return fsVec3d(v.m_x*d,v.m_y*d,v.m_z*d); }
inline fsVec3d operator+(const fsVec3d& a, const fsVec3d& b){ return fsVec3d(a.m_x+b.m_x, a.m_y+b.m_y, a.m_z+b.m_z); }

struct fsRot {
    double m[3][3];
    fsRot(){identity();}
    inline void set(double m[3][3]){
        for(int i=0;i<3;++i)
            for(int j=0;j<3;++j)
                this->m[i][j] = m[i][j];
    }
    void identity();
    void rot_x(double t);
    void rot_y(double t);
    void rot_z(double t);
};

class Kinematics
{
public:

    fsVec3d computePosition(int ch_a, int ch_b, int ch_c){ int enc[3] = {ch_a,ch_b,ch_c}; return computePosition(enc); }
    fsVec3d computePosition(const int* encoderValues); // encoders[3]
    fsVec3d computeBodyAngles(const int* encoderValues); // encoders[3]
    fsVec3d computeMotorAmps(fsVec3d force, const int* encoderValues);
    fsRot computeRotation(const int* encBase, const int* encRot);




    //! A collection of variables that can be set in ~/wooden_haptics.json
    struct configuration {
        double variant;                 // 0=WoodenHaptics default, 1=AluHaptics
        double diameter_capstan_a;      // m
        double diameter_capstan_b;      // m
        double diameter_capstan_c;      // m
        double length_body_a;           // m
        double length_body_b;           // m
        double length_body_c;           // m
        double diameter_body_a;         // m
        double diameter_body_b;         // m
        double diameter_body_c;         // m
        double workspace_origin_x;      // m
        double workspace_origin_y;      // m
        double workspace_origin_z;      // m
        double workspace_radius;        // m (for application information)
        double torque_constant_motor_a; // Nm/A
        double torque_constant_motor_b; // Nm/A
        double torque_constant_motor_c; // Nm/A
        double current_for_10_v_signal; // A
        double cpr_encoder_a;           // quadrupled counts per revolution
        double cpr_encoder_b;           // quadrupled counts per revolution
        double cpr_encoder_c;           // quadrupled counts per revolution
        double max_linear_force;        // N
        double max_linear_stiffness;    // N/m
        double max_linear_damping;      // N/(m/s)
        double mass_body_b;             // Kg
        double mass_body_c;             // Kg
        double length_cm_body_b;        // m     distance to center of mass
        double length_cm_body_c;        // m     from previous body
        double g_constant;              // m/s^2 usually 9.81 or 0 to
                                        //       disable gravity compensation

        std::string name;

        // Set values
        configuration(const double* k, std::string name="unnamed variant"):
          variant(k[0]),
          diameter_capstan_a(k[1]), diameter_capstan_b(k[2]), diameter_capstan_c(k[3]),
          length_body_a(k[4]), length_body_b(k[5]), length_body_c(k[6]),
          diameter_body_a(k[7]), diameter_body_b(k[8]), diameter_body_c(k[9]),
          workspace_origin_x(k[10]), workspace_origin_y(k[11]), workspace_origin_z(k[12]),
          workspace_radius(k[13]), torque_constant_motor_a(k[14]),
          torque_constant_motor_b(k[15]), torque_constant_motor_c(k[16]),
          current_for_10_v_signal(k[17]), cpr_encoder_a(k[18]), cpr_encoder_b(k[19]),
          cpr_encoder_c(k[20]), max_linear_force(k[21]), max_linear_stiffness(k[22]),
          max_linear_damping(k[23]), mass_body_b(k[24]), mass_body_c(k[25]),
          length_cm_body_b(k[26]), length_cm_body_c(k[27]), g_constant(k[28]), name(name){}

        configuration(){}

        // Some configurations (won't require to use the filesystem)
        static configuration woodenhaptics_v2015() {
            double data[] = { 0, 0.010, 0.010, 0.010,
                              0.080, 0.205, 0.245,
                              0.160, 0.120, 0.120,
                              0.220, 0.000, 0.080, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0};
            return Kinematics::configuration(data,"woodenhaptics_v2015 hardcoded");
        }

        static configuration polhem_v1() {
            double data[] = { 2, 0.010, 0.010, 0.010,
                              0.058, 0.174, 0.133,
                              0.180, 0.100, 0.100,
                              0.140, 0.000, 0.100, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                              5.0, 800.0, 8.0,
                              0.080, 0.080, 0.040, 0.070, 0};
            return Kinematics::configuration(data,"polhem_v1 hardcoded");
        }

        static configuration aluhaptics_v2() {
            double data[] = { 1, 0.0138, 0.0098, 0.0098,
                              0.111, 0.140, 0.111,
                              0.116, 0.076, 0.076,
                              0.140, 0.000, 0.000, 0.100,
                              0.0259, 0.0259, 0.0259, 3.0, 2000, 2000, 2000,
                              5.0, 1000.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0};
            return Kinematics::configuration(data,"aluhaptics_v2 hardcoded");
        }

        static configuration vintage() {
            double data[] = { 1, 0.013, 0.010, 0.010,
                              0.056, 0.138, 0.112,
                              0.117, 0.077, 0.077,
                              0.140, 0.000, 0.000, 0.100,
                              0.0163, 0.0163, 0.0163, 3.0, 4000, 4000, 4000,
                              5.0, 800.0, 8.0,
                              0.170, 0.110, 0.051, 0.091, 0};
            return Kinematics::configuration(data, "vintage hardcoded");
        }
    };

    Kinematics();
    Kinematics(configuration c):m_config(c) {}


    const configuration m_config;
};

std::string toJSON(const Kinematics::configuration& c);
Kinematics::configuration fromJSON(std::string json);




inline std::string toString(const fsVec3d& r);
inline std::string toString(const fsRot& r);

inline fsRot operator*(const fsRot& a, const fsRot& b) {
    fsRot c;
    int i,j,m;
    for(i=0;i<3;i++) {
      for(j=0;j<3;j++) {
        c.m[i][j] = 0;
        for(m=0;m<3;m++)
          c.m[i][j] += a.m[i][m]*b.m[m][j];
      }
    }
    return c;
}

class FsHapticDeviceThread;


class HaptikfabrikenInterface {
public:
    HaptikfabrikenInterface(bool wait_for_next_message=false,
                            Kinematics::configuration c=Kinematics::configuration::polhem_v1());


    int open();    // Returns 0 if success, otherwise a error number
    void close();

    // If error, get error message here
    std::string getErrorCode();


    // Get/set methods (thread safe, gets latest known values and sets next to be commanded)
    void getEnc(int a[]);  // Array of 6 elements will be filled with signed encoder values
    fsVec3d getBodyAngles();
    void getLatestCommandedMilliamps(int ma[]); // Array of 3 elements
    int getNumSentMessages(); // usb communications statistics
    int getNumReceivedMessages();

    fsVec3d getPos();              // Get cartesian coords xyz using kineamtics model
    fsRot getRot();                // Get orientaion of manipulandum
    void setForce(fsVec3d f);      // Set force using kineamtics model, e.g. in xyz
    void setCurrent(fsVec3d amps); // Set the 3 motor amps directly (not cartesian coords.)

    int max_milliamps = 2000;      // Cap the current provided to motors

private:
    FsHapticDeviceThread *fsthread;
};

} // namespace haptikfabriken

#endif // HAPTIKFABRIKENAPI_H
