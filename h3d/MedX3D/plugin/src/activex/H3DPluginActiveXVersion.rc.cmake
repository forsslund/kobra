LANGUAGE LANG_ENGLISH, SUBLANG_ENGLISH_US

VS_VERSION_INFO VERSIONINFO
 FILEVERSION ${H3DPluginActiveX_MAJOR_VERSION},${H3DPluginActiveX_MINOR_VERSION},${H3DPluginActiveX_BUILD_VERSION},${H3DPluginActiveX_SVN_VERSION}
 PRODUCTVERSION ${H3DPluginActiveX_MAJOR_VERSION},${H3DPluginActiveX_MINOR_VERSION},${H3DPluginActiveX_BUILD_VERSION},${H3DPluginActiveX_SVN_VERSION}
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
            VALUE "Comments", "An ActiveX plugin to IE. The plugin makes it possible to run H3D API scenes in the browser. With the additional Volume Rendering component.\0"
            VALUE "CompanyName", "SenseGraphics AB\0"
            VALUE "FileDescription", "H3D X3D Player ActiveX Control"
            VALUE "FileVersion", "${H3DPluginActiveX_MAJOR_VERSION}, ${H3DPluginActiveX_MINOR_VERSION}, ${H3DPluginActiveX_BUILD_VERSION}, ${H3DPluginActiveX_SVN_VERSION}\0"
            VALUE "InternalName", "${H3DPluginActiveX_outputName}\0"
            VALUE "LegalCopyright", "Copyright (C) 2008\0"
            VALUE "LegalTrademarks", "www.h3d.org; www.sensegraphics.com\0"
            VALUE "OriginalFilename", "${H3DPluginActiveX_outputName}\0"
            VALUE "ProductName", "H3DAPI activeX plugin\0"
            VALUE "ProductVersion", "${H3DPluginActiveX_MAJOR_VERSION}, ${H3DPluginActiveX_MINOR_VERSION}, ${H3DPluginActiveX_BUILD_VERSION}, ${H3DPluginActiveX_SVN_VERSION}\0"
            VALUE "Build Environment", "${H3D_BUILD_ENVIRONMENT}\0"
        END
    END
    BLOCK "VarFileInfo"
    BEGIN
        VALUE "Translation", 0x409, 1252
    END
END
