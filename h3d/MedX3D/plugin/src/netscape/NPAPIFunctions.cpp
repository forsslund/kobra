

#include <H3D/ProfilesAndComponents.h>

#include "H3DPluginNetscape.h"

#ifdef H3D_WINDOWS
#include "H3DPluginNetscapeWin.h"
#endif

#ifdef H3D_OSX
#include "H3DPluginNetscapeOSX.h"
#endif // H3D_OSX

#ifdef H3D_LINUX
#include "H3DPluginNetscapeLinux.h"
#endif

NPError NS_PluginInitialize()
{
  ProfilesAndComponents::check_profiles_components = false;
#ifdef PRINT_ERR_FILE
  cerr << "NS_PluginInitialize()" << endl;
#endif
  
  // No special initialization required
  return NPERR_NO_ERROR;
}

void NS_PluginShutdown()
{
#ifdef PRINT_ERR_FILE
  cerr << "NS_PluginShutdown" << endl;
  // No special shutdown stuff either
#endif
}

nsPluginInstanceBase* NS_NewPluginInstance( nsPluginCreateData* data )
{
#ifdef PRINT_ERR_FILE
  cerr << "NS_NewPluginInstance" << endl;
#endif
  if( !data )
    return NULL;

  if( controlclass ) {
    controlclass->removeInstanceThread();
    //controlclass->OnDestroy();
  }

#ifdef H3D_WINDOWS
  controlclass = new H3DPluginNetscapeWin( data );
#else
#ifdef H3D_OSX
 controlclass = new H3DPluginNetscapeOSX( data );
#else
#ifdef H3D_LINUX
 controlclass = new H3DPluginNetscapeLinux( data );
#else
 //"Unsupported platform"
#endif
#endif
#endif

#ifdef PRINT_ERR_FILE
  cerr << "NS_NewPluginInstance" << endl;
#endif
  controlclass->setControlClass( true );

  controlclass->printControlClass();

  return controlclass;
}

void NS_DestroyPluginInstance( nsPluginInstanceBase* aPlugin )
{
  H3DPluginNetscape * temp_plg_ntsc = 0;
  if( aPlugin )
    temp_plg_ntsc = (H3DPluginNetscape*)aPlugin;

  if( aPlugin ) {
    if( temp_plg_ntsc && temp_plg_ntsc->getIsControlClass() ) {
      temp_plg_ntsc->setControlClass( false );
      controlclass = 0;
    }
    delete (H3DPluginNetscape*)aPlugin;
  }
}

#if( defined(XP_UNIX) || defined( XP_MACOSX) ) 
char *NPP_GetMIMEDescription() {
  return 
    "model/x3d+xml:x3d:X3D scene;model/x3d+vrml:x3dv:X3D scene;model/vrml:wrl:VRML 2.0 scene";
    
}
#endif


#ifdef XP_UNIX
NPError NS_PluginGetValue(NPPVariable var, void *out) {
  NPError ret = NPERR_NO_ERROR;
  char**  val;
  
  if(out == NULL)
    { return NPERR_INVALID_PARAM; }
  
  val = (char**)(out);
  
  switch(var) {
  case NPPVpluginNameString:
    *val = "H3DViewer plugin";
    break;
  case NPPVpluginDescriptionString:
    *val = "Run X3D based 3D-content in the browser. Inlcudes extensions for haptics.";
    break;
  default:
    ret = NPERR_INVALID_PARAM;
    break;
  }

  return ret;
}

#endif
