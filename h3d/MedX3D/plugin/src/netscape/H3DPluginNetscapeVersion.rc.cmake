LANGUAGE LANG_ENGLISH, SUBLANG_ENGLISH_US

VS_VERSION_INFO VERSIONINFO
 FILEVERSION ${H3DPluginNetscape_MAJOR_VERSION},${H3DPluginNetscape_MINOR_VERSION},${H3DPluginNetscape_BUILD_VERSION},${H3DPluginNetscape_SVN_VERSION}
 PRODUCTVERSION ${H3DPluginNetscape_MAJOR_VERSION},${H3DPluginNetscape_MINOR_VERSION},${H3DPluginNetscape_BUILD_VERSION},${H3DPluginNetscape_SVN_VERSION}
 FILEFLAGSMASK 0x3fL
#ifdef _DEBUG
 FILEFLAGS 0x1L
#else
 FILEFLAGS 0x0L
#endif
 FILEOS 0x4L
 FILETYPE 0x2L
 FILESUBTYPE 0x0L
BEGIN
    BLOCK "StringFileInfo"
    BEGIN
        BLOCK "040904e4"
        BEGIN
            VALUE "Comments", "A plugin for Netscape like browsers (Mozilla Firefox and Opera for example). The plugin makes it possible to run X3D scenes and H3D API scenes in the browser.\0"
            VALUE "CompanyName", "SenseGraphics AB\0"
            VALUE "FileDescription", "H3DViewer plugin\0"
            VALUE "FileVersion", "${H3DPluginNetscape_MAJOR_VERSION}, ${H3DPluginNetscape_MINOR_VERSION}, ${H3DPluginNetscape_BUILD_VERSION}, ${H3DPluginNetscape_SVN_VERSION}\0"
            VALUE "InternalName", "${H3DPluginNetscape_outputName}\0"
            VALUE "LegalCopyright", "Copyright (C) 2008\0"
            VALUE "LegalTrademarks", "www.h3d.org; www.sensegraphics.com\0"
            VALUE "OriginalFilename", "${H3DPluginNetscape_outputName}\0"
            VALUE "ProductName", "H3DAPI netscape plugin\0"
            VALUE "ProductVersion", "${H3DPluginNetscape_MAJOR_VERSION}, ${H3DPluginNetscape_MINOR_VERSION}, ${H3DPluginNetscape_BUILD_VERSION}, ${H3DPluginNetscape_SVN_VERSION}\0"
            VALUE "FileExtents", "x3d|x3dv|wrl\0"
            VALUE "FileOpenName", "X3D scene|X3D scene|VRML 2.0 scene\0"
            VALUE "MIMEType", "model/x3d+xml|model/x3d+vrml|model/vrml\0"
            VALUE "Build Environment", "${H3D_BUILD_ENVIRONMENT}\0"
        END
    END
    BLOCK "VarFileInfo"
    BEGIN
        VALUE "Translation", 0x409, 1252
    END
END
