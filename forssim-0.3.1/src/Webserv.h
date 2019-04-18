#ifndef WEBSERV
#define WEBSERV

#include "server_http.hpp"
#include "client_http.hpp"

#include <H3D/X3DChildNode.h>
#include <H3D/SFString.h>
#include <H3D/SFBool.h>
#include <H3D/MFInt32.h>
#include <H3D/MFString.h>
#include <H3D/SFInt32.h>
#include <H3D/SFFloat.h>
#include <H3D/SFRotation.h>
#include <H3D/SFVec3f.h>
#include <H3D/SFMatrix4f.h>

#include <thread>

using namespace std;
using namespace H3D;

typedef SimpleWeb::Server<SimpleWeb::HTTP> HttpServer;
typedef SimpleWeb::Client<SimpleWeb::HTTP> HttpClient;


//class HttpServer;

namespace FS
{

    class fpsCounter {
        H3D::TimeStamp last_ten_frames;
        int frame_count;
        float fps;

    public:
        fpsCounter() : frame_count(0), fps(0) {}
        float getFps() const { return fps; }
        void tick() {
            if(++frame_count == 10){
                fps = float(10.0/(TimeStamp::now() - last_ten_frames));
                last_ten_frames=TimeStamp::now();
                frame_count = 0;
            }
        }
    };

    /**
     * Concerned with the Pedal of data to file
     */
    class Webserv : public X3DChildNode
    {
    public:
        Webserv(Inst< SFBool  > pedal_0 = 0,
                Inst< SFBool  > pedal_1 = 0,
                Inst< SFBool  > pedal_2 = 0,
                Inst< MFInt32 > noOfVoxelsBoredByUser = 0,
                Inst< MFString> segmentNameField = 0,
                Inst< MFString> forbidden_segmentNameField = 0,
                Inst< MFInt32> forbidden_noOfVoxelsBoredByUser = 0,
                Inst< SFInt32> state = 0,
                Inst< SFInt32> showHead = 0,
                Inst< SFFloat> fractionUserExpertCurrentStep = 0,
                Inst< SFBool> playback_isPlay = 0,
                Inst< SFFloat> playback_time = 0,
                Inst< SFBool> saveVolume = 0,
                Inst< SFRotation> teethRotation = 0,
                Inst< SFBool> doPlayback = 0,
                Inst< SFVec3f> adrillforce_force = 0,
                Inst<SFString> textout = 0,
                Inst< SFVec3f> gpvec3f0 = 0,
                Inst< SFVec3f> gpvec3f1 = 0,
                Inst< SFVec3f> gpvec3f2 = 0,
                Inst< SFVec3f> gpvec3f3 = 0,
                Inst< SFVec3f> gpvec3f4 = 0,
                Inst< SFVec3f> gpvec3f5 = 0,
                Inst< SFVec3f> gpvec3f6 = 0,
                Inst< SFVec3f> gpvec3f7 = 0,
                Inst< SFVec3f> gpvec3f8 = 0,
                Inst< SFVec3f> gpvec3f9 = 0,
                Inst< SFFloat> gpfloat0 = 0,
                Inst< SFFloat> gpfloat1 = 0,
                Inst< SFFloat> gpfloat2 = 0,
                Inst< SFFloat> gpfloat3 = 0,
                Inst< SFFloat> gpfloat4 = 0,
                Inst< SFFloat> gpfloat5 = 0,
                Inst< SFFloat> gpfloat6 = 0,
                Inst< SFFloat> gpfloat7 = 0,
                Inst< SFFloat> gpfloat8 = 0,
                Inst< SFFloat> gpfloat9 = 0,
                Inst< SFMatrix4f> gpmatrix4f0 = 0,
                Inst< SFMatrix4f> gpmatrix4f1 = 0,
                Inst< SFMatrix4f> gpmatrix4f2 = 0,
                Inst< SFMatrix4f> gpmatrix4f3 = 0,
                Inst< SFMatrix4f> gpmatrix4f4 = 0,
                Inst< SFMatrix4f> gpmatrix4f5 = 0,
                Inst< SFMatrix4f> gpmatrix4f6 = 0,
                Inst< SFMatrix4f> gpmatrix4f7 = 0,
                Inst< SFMatrix4f> gpmatrix4f8 = 0,
                Inst< SFMatrix4f> gpmatrix4f9 = 0
                );
        virtual ~Webserv();

        /**
             * Initialize the Pedal Node
             */
        void initialize();

        /**
             * The H3DNodeDatabase for this node
             */
        static H3DNodeDatabase database;

        virtual void traverseSG( H3D::TraverseInfo &ti );

        int fd;

        auto_ptr<SFBool> pedal_0;
        auto_ptr<SFBool> pedal_1;
        auto_ptr<SFBool> pedal_2;
        auto_ptr<MFInt32> noOfVoxelsBoredByUser;
        auto_ptr<MFString> segmentNameField;
        auto_ptr<MFString> forbidden_segmentNameField;
        auto_ptr<MFInt32> forbidden_noOfVoxelsBoredByUser;
        auto_ptr<SFInt32> state;
        auto_ptr<SFInt32> showHead;
        auto_ptr<SFFloat> fractionUserExpertCurrentStep;
        auto_ptr<SFBool> playback_isPlay;
        auto_ptr<SFFloat> playback_time;
        auto_ptr<SFBool> saveVolume;
        auto_ptr<SFRotation> teethRotation;
        auto_ptr<SFBool> doPlayback;
        auto_ptr<SFVec3f> adrillforce_force;
        auto_ptr<SFString> textout;

        auto_ptr<SFVec3f> gpvec3f0;
        auto_ptr<SFVec3f> gpvec3f1;
        auto_ptr<SFVec3f> gpvec3f2;
        auto_ptr<SFVec3f> gpvec3f3;
        auto_ptr<SFVec3f> gpvec3f4;
        auto_ptr<SFVec3f> gpvec3f5;
        auto_ptr<SFVec3f> gpvec3f6;
        auto_ptr<SFVec3f> gpvec3f7;
        auto_ptr<SFVec3f> gpvec3f8;
        auto_ptr<SFVec3f> gpvec3f9;
        auto_ptr<SFFloat> gpfloat0;
        auto_ptr<SFFloat> gpfloat1;
        auto_ptr<SFFloat> gpfloat2;
        auto_ptr<SFFloat> gpfloat3;
        auto_ptr<SFFloat> gpfloat4;
        auto_ptr<SFFloat> gpfloat5;
        auto_ptr<SFFloat> gpfloat6;
        auto_ptr<SFFloat> gpfloat7;
        auto_ptr<SFFloat> gpfloat8;
        auto_ptr<SFFloat> gpfloat9;
        auto_ptr<SFMatrix4f> gpmatrix4f0;
        auto_ptr<SFMatrix4f> gpmatrix4f1;
        auto_ptr<SFMatrix4f> gpmatrix4f2;
        auto_ptr<SFMatrix4f> gpmatrix4f3;
        auto_ptr<SFMatrix4f> gpmatrix4f4;
        auto_ptr<SFMatrix4f> gpmatrix4f5;
        auto_ptr<SFMatrix4f> gpmatrix4f6;
        auto_ptr<SFMatrix4f> gpmatrix4f7;
        auto_ptr<SFMatrix4f> gpmatrix4f8;
        auto_ptr<SFMatrix4f> gpmatrix4f9;



        void jonas_reply (HttpServer::Response& response, shared_ptr<HttpServer::Request> request);

    private:
        thread webservThread;
        HttpServer* server;
        void start() { server->start(); }
        fpsCounter fc;


    };
}


#endif // WEBSERV

