# README
This folder contains a demo to show the features of MedX3D and plugins for Netscape like browsers and Internet explorer.


## Installation Instructions
Run the installation program downloaded from www.h3d.org.


## Plugins
The following installation information is only needed if it is desired to register the plugin manually.  
First you need to make sure that the distributed versions in the bin folder can be found by the plugin.  
Add the folder where these dll-s reside to the environment variable `PATH`. (Or move them to a folder which is already in the path).  
In case there are other versions of the distributed dll-s on your system make sure that the ones provided are the one the plugin is trying to load when active.

### Internet Explorer
To register the plugin for IE start the command prompt.  
Type `regsvr32 $(PATH_TO_FOLDER_CONTAINING_PLUGIN)\H3DPluginActiveX.ocx`  
X3D-files will now be run using this plugin in internet explorer. I think that it at the moment replaces any existing plugin using x3d-files.  
To unregister the activeX plugin start the command prompt and type `regsvr32 /u $(PATH_TO_FOLDER_CONTAINING_PLUGIN)\H3DPluginActiveX.ocx`.

### Netscape plugin (Mozilla FireFox, Opera )
Move the npH3D32.dll to the appropriate folder for the browser you are using.  
Opera: `$(InstallDirectory)\program\plugins`  
FireFox: `$(InstallDirectory)\plugins`


## Features
Right click in the window to bring up a navigation menu and a menu for choosing viewpoint.
Navigation is done using keyboard and/or mouse.
Note that the files loaded can contains Python scripting (if they contain H3D content).
This is a security risk, NEVER open files on untrusted sites.


## Known bugs
The menus might get screwed up in Opera when several plugin instances are started.

Only one instance at a time will be active.  
When switching between them refresh needs to be done to load the plugin again.  
Of course different browsers can be open at the same time.   
Different instances of IE also behaves correct.  
**IMPORTANT:** this version of the plugin will not display error messages.  

When unregistering the ActiveX dll it might sometimes give a failed message.  
This is most probably caused by some problem with deleting information about a certain mime-type.  
Even if this error is displayed the plugin will not work anymore.


## Note for developers
Two extra convenience projects are created by the CMakeLists.txt in order to
easily register and unregister the created ActiveX plugin without having to
do this manually with regsvr32 in the command prompt. Note however that
since CMAKE seems to not provide the functionality of specifying post build
event for each configuration the registered ActiveX plugin will always be the
one for release build. This could be fixed by creating an executable that takes
arguments but the necessity for this is probably very small.

For developers of the Netscape plugin it might be convenient in Windows 
to create the environment variable `MOZ_PLUGIN_PATH` and set it to the path where the
plugin is installed when the INSTALL project is built. This will let Mozilla Firefox detect the plugin
without moving it to the plugin folder in the install location of the browser.