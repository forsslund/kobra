#include "Webserv.h"


//Added for the json-example
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//Added for the default_resource example
#include<fstream>
#include <functional>

using namespace FS;
using namespace std;
//Added for the json-example:
using namespace boost::property_tree;





/// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase Webserv::database("Webserv",
        &(newInstance<Webserv>),
        typeid( Webserv ),
        &X3DChildNode::database );

namespace WebservInternals
{
    H3D::FIELDDB_ELEMENT( Webserv, pedal_0, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, pedal_1, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, pedal_2, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, noOfVoxelsBoredByUser, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, segmentNameField, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, forbidden_segmentNameField, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, forbidden_noOfVoxelsBoredByUser, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, state, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, fractionUserExpertCurrentStep, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, playback_isPlay, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, playback_time, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, showHead, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, saveVolume, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, teethRotation, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, doPlayback, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, adrillforce_force, INPUT_OUTPUT );
    H3D::FIELDDB_ELEMENT( Webserv, textout, INPUT_OUTPUT );
}

Webserv::Webserv(Inst<SFBool> pedal_0,
                 Inst<SFBool> pedal_1,
                 Inst<SFBool> pedal_2,
                 Inst<MFInt32> noOfVoxelsBoredByUser,
                 Inst<MFString> segmentNameField,
                 Inst<MFString> forbidden_segmentNameField,
                 Inst< MFInt32> forbidden_noOfVoxelsBoredByUser,
                 Inst< SFInt32> state,
                 Inst< SFInt32> showHead,
                 Inst< SFFloat> fractionUserExpertCurrentStep,
                 Inst< SFBool> playback_isPlay,
                 Inst< SFFloat> playback_time,
                 Inst< SFBool> saveVolume,
                 Inst< SFRotation> teethRotation,
                 Inst< SFBool> doPlayback,
                 Inst< SFVec3f> adrillforce_force,
                 Inst<SFString> textout):
                    pedal_0(pedal_0),
                    pedal_1(pedal_1),
                    pedal_2(pedal_2),
                    noOfVoxelsBoredByUser(noOfVoxelsBoredByUser),
                    segmentNameField(segmentNameField),
                    forbidden_segmentNameField(forbidden_segmentNameField),
                    forbidden_noOfVoxelsBoredByUser(forbidden_noOfVoxelsBoredByUser),
                    state(state),
                    showHead(showHead),
                    fractionUserExpertCurrentStep(fractionUserExpertCurrentStep),
                    playback_isPlay(playback_isPlay),
                    playback_time(playback_time),
                    saveVolume(saveVolume),
                    teethRotation(teethRotation),
                    doPlayback(doPlayback),
                    adrillforce_force(adrillforce_force),
                    textout(textout){
    cout << "Webserv Constructing"<<endl;

    type_name = "Webserv";
    database.initFields( this );




    cout << "Webserv Constructed"<<endl;
}

Webserv::~Webserv()
{
    cout<<"Webserv Stopping Server"<<endl;
    if(webservThread.joinable()){
        server->stop();
        webservThread.join();
    }
    cout<<"Webserv Node Destructed"<<endl;
}

void Webserv::traverseSG( H3D::TraverseInfo&){
    fc.tick();

}

void Webserv::jonas_reply(HttpServer::Response& response,
                          shared_ptr<HttpServer::Request> request)
{
  string path=request->path;
  if(path=="/head/off")
    showHead->setValue(1);
  if(path=="/head/on")
    showHead->setValue(0);
  if(path=="/system/exit"){
    std::cout << "**********************" << std::endl
              << "Exiting application..." << std::endl;
    throw Exception::QuitAPI();
  }
  if(path=="/playback/play")
    doPlayback->setValue(true);
  if(path=="/playback/pause")
    doPlayback->setValue(false);
  if(path=="/save")
    saveVolume->setValue(true);
  if(path.find("/teeth/rotation/") != string::npos){
    string deg = path.substr(16,string::npos);
    double rad = atoi(deg.c_str()) * 3.1415/180.0;
    teethRotation->setValue(Rotation(0,1,0,float(rad)));

  }






    stringstream content;
    content << "{";


    content << "\"request_path\": " << "\"" << request->path << "\"" << ",";

    content << "\"fps\": " << fc.getFps() << ",";

    // Materials list
    // NOTE: id is confusingly set to i+1 here. That is because the GUI
    //       thought that was how we indexed them. "Air" is then id 1.
    //       should be reverted to indexed from 0 in the future.
    content << "\"materials\": [ ";
    const vector<string>& segmentNames = segmentNameField->getValue();
    const vector<int>& val = noOfVoxelsBoredByUser->getValue();
    for(int i=0;i<segmentNames.size();++i){
        content << "{" <<
                             "\"id\": " << i+1 << "," <<
                             "\"name\": \"" << segmentNames[i] << "\"," <<
                             "\"removed_voxels\": { \"total\": " <<  val[i] << "}"<<
                          "}";
        if(i!=segmentNames.size()-1)
            content << ",";
    }
    content << "],";

    // Forbidden list
    content << "\"forbidden\": [ ";
    const vector<string>& fNames = forbidden_segmentNameField->getValue();
    const vector<int>& fVal = forbidden_noOfVoxelsBoredByUser->getValue();
    for(int i=0;i<fNames.size();++i){
        content << "{" <<
                     "\"id\": " << i+1 << "," <<
                     "\"name\": \"" << fNames[i] << "\"," <<
                     "\"removed_voxels\": { \"forbidden\": " << fVal[i] << "}"<<
                   "}";
        if(i!=fNames.size()-1)
            content << ",";
    }
    content << "],";

    content << "\"current_state\": " << state->getValue() << ",";
    content << "\"fraction_user_expert_current_step\": " <<
               fractionUserExpertCurrentStep->getValue() << ",";
    content << "\"playback_is_play\": " << (playback_isPlay->getValue()? 1:0) << ",";
    content << "\"playback_time\": " << playback_time->getValue() << ",";
    content << "\"showHead\": " << showHead->getValue() << ",";
    content << textout->getValue(); // format:    "key: value,"
    content << "\"forssim_build_date\": \"" << std::string(__DATE__) << " " <<
                                               std::string(__TIME__) << "\"";

    content << "}";


    //find length of content_stream (length received using content_stream.tellp())
    content.seekp(0, ios::end);

    response <<  "HTTP/1.1 200 OK\r\n"<<
                 "Content-Length: " << content.tellp() << "\r\n" <<
                 "Access-Control-Allow-Origin: *\r\n" <<
                 "\r\n" << content.rdbuf();


}

void Webserv::initialize()
{
    cout<<"Webserv Node Initializing"<<endl;
    pedal_0->setValue(false);
    pedal_1->setValue(false);
    pedal_2->setValue(false);



















    //HTTP-server at port 8080 using 4 threads
    server = new HttpServer(8080, 1);



    using namespace std::placeholders;

    std::function<void(HttpServer::Response&,
                       std::shared_ptr<HttpServer::Request>)> f =
        std::bind(&Webserv::jonas_reply, this, placeholders::_1,
                                               placeholders::_2);

    //server->resource["^/$"]["GET"] = f;
    server->default_resource["GET"] = f;

//    server->resource["^/head/off$"]["GET"] = f;

    /*
    //Add resources using path-regex and method-string, and an anonymous function
    //POST-example for the path /string, responds the posted string
    server->resource["^/string$"]["POST"]=[](HttpServer::Response& response, shared_ptr<HttpServer::Request> request) {
        //Retrieve string from istream (request->content)
        stringstream ss;
        request->content >> ss.rdbuf();
        string content=ss.str();

        response << "HTTP/1.1 200 OK\r\nContent-Length: " << content.length() << "\r\n\r\n" << content;
    };

    //POST-example for the path /json, responds firstName+" "+lastName from the posted json
    //Responds with an appropriate error message if the posted json is not valid, or if firstName or lastName is missing
    //Example posted json:
    //{
    //  "firstName": "John",
    //  "lastName": "Smith",
    //  "age": 25
    //}
    server->resource["^/json$"]["POST"]=[](HttpServer::Response& response, shared_ptr<HttpServer::Request> request) {
        try {
            ptree pt;
            read_json(request->content, pt);

            string name=pt.get<string>("firstName")+" "+pt.get<string>("lastName");

            std::cout << "The JSON name is: " << name << std::endl;

            response << "HTTP/1.1 200 OK\r\nContent-Length: " << name.length() << "\r\n\r\n" << name;
        }
        catch(exception& e) {
            response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << strlen(e.what()) << "\r\n\r\n" << e.what();
        }
    };

    //GET-example for the path /match/[number], responds with the matched string in path (number)
    //For instance a request GET /match/123 will receive: 123
    server->resource["^/match/([0-9]+)$"]["GET"]=[](HttpServer::Response& response,
            shared_ptr<HttpServer::Request> request) {
        string number=request->path_match[1];
        response << "HTTP/1.1 200 OK\r\nContent-Length: " << number.length() << "\r\n\r\n" << number;
    };

    //Default GET-example. If no other matches, this anonymous function will be called.
    //Will respond with content in the web/-directory, and its subdirectories.
    //Default file: index.html
    //Can for instance be used to retrieve an HTML 5 client that uses REST-resources on this server
    server->default_resource["GET"]=[](HttpServer::Response& response, shared_ptr<HttpServer::Request> request) {
        string filename="web";

        string path=request->path;

        //Replace all ".." with "." (so we can't leave the web-directory)
        size_t pos;
        while((pos=path.find(".."))!=string::npos) {
            path.erase(pos, 1);
        }

        filename+=path;
        ifstream ifs;
        //A simple platform-independent file-or-directory check do not exist, but this works in most of the cases:
        if(filename.find('.')==string::npos) {
            if(filename[filename.length()-1]!='/')
                filename+='/';
            filename+="index.html";
        }
        ifs.open(filename, ifstream::in);

        if(ifs) {
            ifs.seekg(0, ios::end);
            size_t length=ifs.tellg();

            ifs.seekg(0, ios::beg);

            response << "HTTP/1.1 200 OK\r\nContent-Length: " << length << "\r\n\r\n";

            //read and send 128 KB at a time if file-size>buffer_size
            size_t buffer_size=131072;
            if(length>buffer_size) {
                vector<char> buffer(buffer_size);
                size_t read_length;
                while((read_length=ifs.read(&buffer[0], buffer_size).gcount())>0) {
                    response.stream.write(&buffer[0], read_length);
                    response << HttpServer::flush;
                }
            }
            else
                response << ifs.rdbuf();

            ifs.close();
        }
        else {
            string content="Could not open file "+filename;
            response << "HTTP/1.1 400 Bad Request\r\nContent-Length: " << content.length() << "\r\n\r\n" << content;
        }
    };
    */

    webservThread = std::thread(&FS::Webserv::start,this);
    cout<<"Webserv Node Initialized"<<endl;


    return;
}

