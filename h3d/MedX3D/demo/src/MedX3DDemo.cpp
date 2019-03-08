///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Nov  6 2013)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "MedX3DDemo.h"

///////////////////////////////////////////////////////////////////////////

MainFrame::MainFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	MainMenuBar = new wxMenuBar( 0 );
	FileMenu = new wxMenu();
	wxMenuItem* LoadVolumeData;
	LoadVolumeData = new wxMenuItem( FileMenu, wxID_ANY, wxString( wxT("Load volume data..") ) , wxEmptyString, wxITEM_NORMAL );
	FileMenu->Append( LoadVolumeData );
	
	wxMenuItem* LoadRawData;
	LoadRawData = new wxMenuItem( FileMenu, wxID_ANY, wxString( wxT("Load raw data..") ) , wxEmptyString, wxITEM_NORMAL );
	FileMenu->Append( LoadRawData );
	
	FileMenu->AppendSeparator();
	
	wxMenuItem* QuitProgram;
	QuitProgram = new wxMenuItem( FileMenu, wxID_ANY, wxString( wxT("Quit") ) , wxEmptyString, wxITEM_NORMAL );
	FileMenu->Append( QuitProgram );
	
	MainMenuBar->Append( FileMenu, wxT("File") ); 
	
	WindowsMenu = new wxMenu();
	VolumeStyleMenuItem = new wxMenuItem( WindowsMenu, wxID_ANY, wxString( wxT("Render options..") ) , wxEmptyString, wxITEM_CHECK );
	WindowsMenu->Append( VolumeStyleMenuItem );
	
	ConsoleWindowsMenu = new wxMenuItem( WindowsMenu, wxID_ANY, wxString( wxT("Console") ) , wxEmptyString, wxITEM_CHECK );
	WindowsMenu->Append( ConsoleWindowsMenu );
	
	MainMenuBar->Append( WindowsMenu, wxT("Windows") ); 
	
	this->SetMenuBar( MainMenuBar );
	
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( MainFrame::OnClose ) );
	this->Connect( wxEVT_IDLE, wxIdleEventHandler( MainFrame::OnIdle ) );
	this->Connect( LoadVolumeData->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnLoadVolumeData ) );
	this->Connect( LoadRawData->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnLoadRawData ) );
	this->Connect( QuitProgram->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnQuit ) );
	this->Connect( VolumeStyleMenuItem->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnColorVolumeStyleMenuItem ) );
	this->Connect( ConsoleWindowsMenu->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnConsoleWindowsMenu ) );
}

MainFrame::~MainFrame()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( MainFrame::OnClose ) );
	this->Disconnect( wxEVT_IDLE, wxIdleEventHandler( MainFrame::OnIdle ) );
	this->Disconnect( wxID_ANY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnLoadVolumeData ) );
	this->Disconnect( wxID_ANY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnLoadRawData ) );
	this->Disconnect( wxID_ANY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnQuit ) );
	this->Disconnect( wxID_ANY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnColorVolumeStyleMenuItem ) );
	this->Disconnect( wxID_ANY, wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler( MainFrame::OnConsoleWindowsMenu ) );
	
}

MainDialog::MainDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxVERTICAL );
	
	m_notebook2 = new wxNotebook( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0 );
	m_panel3 = new wxPanel( m_notebook2, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	m_panel3->SetToolTip( wxT("Save the current volume data in Nrrd format.") );
	
	wxBoxSizer* bSizer28;
	bSizer28 = new wxBoxSizer( wxVERTICAL );
	
	DensityDataStaticBoxSizer = new wxStaticBoxSizer( new wxStaticBox( m_panel3, wxID_ANY, wxT("Volume data") ), wxVERTICAL );
	
	wxBoxSizer* bSizer19;
	bSizer19 = new wxBoxSizer( wxHORIZONTAL );
	
	LoadButton = new wxButton( m_panel3, wxID_ANY, wxT("Load.."), wxDefaultPosition, wxDefaultSize, 0 );
	LoadButton->SetToolTip( wxT("Load a volume data set to be displayed. Available formats are DICOM, Nrrd, VTK.") );
	
	bSizer19->Add( LoadButton, 0, wxALL, 5 );
	
	LoadRawButton = new wxButton( m_panel3, wxID_ANY, wxT("Load raw.."), wxDefaultPosition, wxDefaultSize, 0 );
	LoadRawButton->SetToolTip( wxT("Load a raw binary dataset allowing you to specify all parameters of the data manually.") );
	
	bSizer19->Add( LoadRawButton, 0, wxALL, 5 );
	
	SaveAsNrrdButton = new wxButton( m_panel3, wxID_ANY, wxT("Save as Nrrd.."), wxDefaultPosition, wxDefaultSize, 0 );
	SaveAsNrrdButton->Enable( false );
	
	bSizer19->Add( SaveAsNrrdButton, 0, wxALL, 5 );
	
	
	DensityDataStaticBoxSizer->Add( bSizer19, 0, 0, 5 );
	
	wxBoxSizer* bSizer20;
	bSizer20 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText3 = new wxStaticText( m_panel3, wxID_ANY, wxT("Loaded file:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText3->Wrap( -1 );
	bSizer20->Add( m_staticText3, 0, wxALL, 5 );
	
	DensityDataLoadedFileText = new wxStaticText( m_panel3, wxID_ANY, wxT("None"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataLoadedFileText->Wrap( -1 );
	bSizer20->Add( DensityDataLoadedFileText, 0, wxALL, 5 );
	
	
	DensityDataStaticBoxSizer->Add( bSizer20, 0, 0, 5 );
	
	DensityDataInfoSizer = new wxGridSizer( 2, 2, 0, 0 );
	
	wxGridSizer* gSizer3;
	gSizer3 = new wxGridSizer( 7, 2, 0, 0 );
	
	m_staticText10 = new wxStaticText( m_panel3, wxID_ANY, wxT("Dimensions:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText10->Wrap( -1 );
	gSizer3->Add( m_staticText10, 0, wxALL, 5 );
	
	DensityDataDimensionsText = new wxStaticText( m_panel3, wxID_ANY, wxT("256x256x64"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataDimensionsText->Wrap( -1 );
	gSizer3->Add( DensityDataDimensionsText, 0, wxALL, 5 );
	
	m_staticText12 = new wxStaticText( m_panel3, wxID_ANY, wxT("Voxel size(in m):"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText12->Wrap( -1 );
	gSizer3->Add( m_staticText12, 0, wxALL, 5 );
	
	DensityDataVoxelSizeText = new wxStaticText( m_panel3, wxID_ANY, wxT("0.01x0.01x0.01"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataVoxelSizeText->Wrap( -1 );
	gSizer3->Add( DensityDataVoxelSizeText, 0, wxALL, 5 );
	
	m_staticText22 = new wxStaticText( m_panel3, wxID_ANY, wxT("Components:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText22->Wrap( -1 );
	gSizer3->Add( m_staticText22, 0, wxALL, 5 );
	
	DensityDataComponentsText = new wxStaticText( m_panel3, wxID_ANY, wxT("LUMINANCE"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataComponentsText->Wrap( -1 );
	gSizer3->Add( DensityDataComponentsText, 0, wxALL, 5 );
	
	m_staticText101 = new wxStaticText( m_panel3, wxID_ANY, wxT("Bits per voxel:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText101->Wrap( -1 );
	gSizer3->Add( m_staticText101, 0, wxALL, 5 );
	
	DensityDataBitsPerVoxelText = new wxStaticText( m_panel3, wxID_ANY, wxT("12"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataBitsPerVoxelText->Wrap( -1 );
	gSizer3->Add( DensityDataBitsPerVoxelText, 0, wxALL, 5 );
	
	m_staticText121 = new wxStaticText( m_panel3, wxID_ANY, wxT("Type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText121->Wrap( -1 );
	gSizer3->Add( m_staticText121, 0, wxALL, 5 );
	
	DensityDataTypeText = new wxStaticText( m_panel3, wxID_ANY, wxT("UNSIGNED"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataTypeText->Wrap( -1 );
	gSizer3->Add( DensityDataTypeText, 0, wxALL, 5 );
	
	m_staticText51 = new wxStaticText( m_panel3, wxID_ANY, wxT("Min value:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText51->Wrap( -1 );
	gSizer3->Add( m_staticText51, 0, wxALL, 5 );
	
	DensityDataMinValueText = new wxStaticText( m_panel3, wxID_ANY, wxT("31000"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataMinValueText->Wrap( -1 );
	gSizer3->Add( DensityDataMinValueText, 0, wxALL, 5 );
	
	m_staticText53 = new wxStaticText( m_panel3, wxID_ANY, wxT("Max value:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText53->Wrap( -1 );
	gSizer3->Add( m_staticText53, 0, wxALL, 5 );
	
	DensityDataMaxValueText = new wxStaticText( m_panel3, wxID_ANY, wxT("33000"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataMaxValueText->Wrap( -1 );
	gSizer3->Add( DensityDataMaxValueText, 0, wxALL, 5 );
	
	
	DensityDataInfoSizer->Add( gSizer3, 1, wxEXPAND, 5 );
	
	wxGridSizer* gSizer31;
	gSizer31 = new wxGridSizer( 3, 2, 0, 0 );
	
	
	DensityDataInfoSizer->Add( gSizer31, 0, 0, 5 );
	
	
	DensityDataStaticBoxSizer->Add( DensityDataInfoSizer, 0, 0, 5 );
	
	
	bSizer28->Add( DensityDataStaticBoxSizer, 0, wxEXPAND, 5 );
	
	
	m_panel3->SetSizer( bSizer28 );
	m_panel3->Layout();
	bSizer28->Fit( m_panel3 );
	m_notebook2->AddPage( m_panel3, wxT("Volume info"), true );
	RenderOptionsPanel = new wxPanel( m_notebook2, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	VolumeDataSizer = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* GeneralSettingsStaticBoxSizer;
	GeneralSettingsStaticBoxSizer = new wxStaticBoxSizer( new wxStaticBox( RenderOptionsPanel, wxID_ANY, wxT("General settings") ), wxVERTICAL );
	
	wxBoxSizer* bSizer58;
	bSizer58 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText58 = new wxStaticText( RenderOptionsPanel, wxID_ANY, wxT("Data filter:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText58->Wrap( -1 );
	bSizer58->Add( m_staticText58, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString m_choice8Choices[] = { wxT("NEAREST"), wxT("LINEAR"), wxT("CUBIC_B_SPLINE"), wxT("CATMULL_ROM") };
	int m_choice8NChoices = sizeof( m_choice8Choices ) / sizeof( wxString );
	m_choice8 = new wxChoice( RenderOptionsPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_choice8NChoices, m_choice8Choices, 0 );
	m_choice8->SetSelection( 1 );
	bSizer58->Add( m_choice8, 0, wxALL, 5 );
	
	
	GeneralSettingsStaticBoxSizer->Add( bSizer58, 0, 0, 5 );
	
	wxBoxSizer* bSizer60;
	bSizer60 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText60 = new wxStaticText( RenderOptionsPanel, wxID_ANY, wxT("Renderer:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText60->Wrap( -1 );
	bSizer60->Add( m_staticText60, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString RendererChoiceChoices[] = { wxT("Ray caster"), wxT("View aligned slices") };
	int RendererChoiceNChoices = sizeof( RendererChoiceChoices ) / sizeof( wxString );
	RendererChoice = new wxChoice( RenderOptionsPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, RendererChoiceNChoices, RendererChoiceChoices, 0 );
	RendererChoice->SetSelection( 0 );
	bSizer60->Add( RendererChoice, 0, wxALL, 5 );
	
	
	GeneralSettingsStaticBoxSizer->Add( bSizer60, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer61;
	bSizer61 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText61 = new wxStaticText( RenderOptionsPanel, wxID_ANY, wxT("Background color:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText61->Wrap( -1 );
	bSizer61->Add( m_staticText61, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_colourPicker19 = new wxColourPickerCtrl( RenderOptionsPanel, wxID_ANY, *wxBLACK, wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	bSizer61->Add( m_colourPicker19, 0, wxALL, 5 );
	
	
	GeneralSettingsStaticBoxSizer->Add( bSizer61, 1, wxEXPAND, 5 );
	
	
	VolumeDataSizer->Add( GeneralSettingsStaticBoxSizer, 0, wxEXPAND, 5 );
	
	wxStaticBoxSizer* RayCasterOptionsStaticBoxSizer;
	RayCasterOptionsStaticBoxSizer = new wxStaticBoxSizer( new wxStaticBox( RenderOptionsPanel, wxID_ANY, wxT("Ray caster options") ), wxVERTICAL );
	
	wxBoxSizer* bSizer59;
	bSizer59 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText59 = new wxStaticText( RenderOptionsPanel, wxID_ANY, wxT("Ray step:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText59->Wrap( -1 );
	bSizer59->Add( m_staticText59, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	RayStepText = new wxTextCtrl( RenderOptionsPanel, wxID_ANY, wxT("0.01"), wxPoint( -1,-1 ), wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	RayStepText->SetMaxLength( 0 ); 
	bSizer59->Add( RayStepText, 0, wxALL, 5 );
	
	
	RayCasterOptionsStaticBoxSizer->Add( bSizer59, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer57;
	bSizer57 = new wxBoxSizer( wxHORIZONTAL );
	
	m_checkBox13 = new wxCheckBox( RenderOptionsPanel, wxID_ANY, wxT("Use empty space skipping"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer57->Add( m_checkBox13, 0, wxALL, 5 );
	
	m_checkBox15 = new wxCheckBox( RenderOptionsPanel, wxID_ANY, wxT("Show non empty space"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer57->Add( m_checkBox15, 0, wxALL, 5 );
	
	
	RayCasterOptionsStaticBoxSizer->Add( bSizer57, 0, 0, 5 );
	
	wxBoxSizer* bSizer56;
	bSizer56 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText57 = new wxStaticText( RenderOptionsPanel, wxID_ANY, wxT("Empty space skipping resolution:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText57->Wrap( -1 );
	bSizer56->Add( m_staticText57, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString m_choice7Choices[] = { wxT("2"), wxT("4"), wxT("8"), wxT("16"), wxT("32") };
	int m_choice7NChoices = sizeof( m_choice7Choices ) / sizeof( wxString );
	m_choice7 = new wxChoice( RenderOptionsPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_choice7NChoices, m_choice7Choices, 0 );
	m_choice7->SetSelection( 2 );
	bSizer56->Add( m_choice7, 0, wxALL, 5 );
	
	
	RayCasterOptionsStaticBoxSizer->Add( bSizer56, 0, 0, 5 );
	
	m_checkBox14 = new wxCheckBox( RenderOptionsPanel, wxID_ANY, wxT("Stop rays at geometries"), wxDefaultPosition, wxDefaultSize, 0 );
	m_checkBox14->SetValue(true); 
	RayCasterOptionsStaticBoxSizer->Add( m_checkBox14, 0, wxALL, 5 );
	
	m_checkBox16 = new wxCheckBox( RenderOptionsPanel, wxID_ANY, wxT("Use stochastic jittering"), wxDefaultPosition, wxDefaultSize, 0 );
	RayCasterOptionsStaticBoxSizer->Add( m_checkBox16, 0, wxALL, 5 );
	
	
	VolumeDataSizer->Add( RayCasterOptionsStaticBoxSizer, 0, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer17;
	sbSizer17 = new wxStaticBoxSizer( new wxStaticBox( RenderOptionsPanel, wxID_ANY, wxT("Slice renderer options") ), wxVERTICAL );
	
	wxBoxSizer* bSizer591;
	bSizer591 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText591 = new wxStaticText( RenderOptionsPanel, wxID_ANY, wxT("Nr slices:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText591->Wrap( -1 );
	bSizer591->Add( m_staticText591, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	NrSlicesText = new wxTextCtrl( RenderOptionsPanel, wxID_ANY, wxT("100"), wxPoint( -1,-1 ), wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	NrSlicesText->SetMaxLength( 0 ); 
	bSizer591->Add( NrSlicesText, 0, wxALL, 5 );
	
	
	sbSizer17->Add( bSizer591, 0, wxEXPAND, 5 );
	
	
	VolumeDataSizer->Add( sbSizer17, 0, wxEXPAND, 5 );
	
	
	RenderOptionsPanel->SetSizer( VolumeDataSizer );
	RenderOptionsPanel->Layout();
	VolumeDataSizer->Fit( RenderOptionsPanel );
	m_notebook2->AddPage( RenderOptionsPanel, wxT("Render options"), false );
	RenderStylesPanel = new wxPanel( m_notebook2, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	RenderStylesSizer = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer26;
	sbSizer26 = new wxStaticBoxSizer( new wxStaticBox( RenderStylesPanel, wxID_ANY, wxT("Type:") ), wxHORIZONTAL );
	
	m_staticText78 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("Type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText78->Wrap( -1 );
	sbSizer26->Add( m_staticText78, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString m_choice18Choices[] = { wxT("Normal"), wxT("Segmented"), wxT("Iso surfaces") };
	int m_choice18NChoices = sizeof( m_choice18Choices ) / sizeof( wxString );
	m_choice18 = new wxChoice( RenderStylesPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_choice18NChoices, m_choice18Choices, 0 );
	m_choice18->SetSelection( 0 );
	sbSizer26->Add( m_choice18, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_button101 = new wxButton( RenderStylesPanel, wxID_ANY, wxT("Style editor.."), wxDefaultPosition, wxDefaultSize, 0 );
	m_button101->SetToolTip( wxT("Open style editor allowing you to change style paramters.") );
	
	sbSizer26->Add( m_button101, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_button14 = new wxButton( RenderStylesPanel, wxID_ANY, wxT("Save as X3D.."), wxDefaultPosition, wxDefaultSize, 0 );
	m_button14->SetToolTip( wxT("Save the current volume rendering scene as an X3D file.") );
	
	sbSizer26->Add( m_button14, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	RenderStylesSizer->Add( sbSizer26, 0, wxEXPAND, 5 );
	
	VolumeDataNodeSizer = new wxStaticBoxSizer( new wxStaticBox( RenderStylesPanel, wxID_ANY, wxT("Volume data") ), wxVERTICAL );
	
	wxBoxSizer* bSizer20211;
	bSizer20211 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText222121 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("Style:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText222121->Wrap( -1 );
	bSizer20211->Add( m_staticText222121, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5 );
	
	wxArrayString VolumeDataRenderStyleChoiceChoices;
	VolumeDataRenderStyleChoice = new wxChoice( RenderStylesPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, VolumeDataRenderStyleChoiceChoices, 0 );
	VolumeDataRenderStyleChoice->SetSelection( 0 );
	VolumeDataRenderStyleChoice->SetToolTip( wxT("The volume rendering style to use.") );
	
	bSizer20211->Add( VolumeDataRenderStyleChoice, 0, wxALL, 5 );
	
	
	VolumeDataNodeSizer->Add( bSizer20211, 0, 0, 5 );
	
	
	RenderStylesSizer->Add( VolumeDataNodeSizer, 0, wxEXPAND, 5 );
	
	SegmentedDataNodeSizer = new wxStaticBoxSizer( new wxStaticBox( RenderStylesPanel, wxID_ANY, wxT("Segmented data") ), wxVERTICAL );
	
	m_button9 = new wxButton( RenderStylesPanel, wxID_ANY, wxT("Load segmentation data.."), wxDefaultPosition, wxDefaultSize, 0 );
	SegmentedDataNodeSizer->Add( m_button9, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer201;
	bSizer201 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText31 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("Loaded file:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText31->Wrap( -1 );
	bSizer201->Add( m_staticText31, 0, wxALL, 5 );
	
	LoadedSegmentDataText = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("None"), wxDefaultPosition, wxDefaultSize, 0 );
	LoadedSegmentDataText->Wrap( -1 );
	bSizer201->Add( LoadedSegmentDataText, 0, wxALL, 5 );
	
	
	SegmentedDataNodeSizer->Add( bSizer201, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer20111;
	bSizer20111 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText3111 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("Segment:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText3111->Wrap( -1 );
	bSizer20111->Add( m_staticText3111, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString SegmentChoiceChoices[] = { wxT("0"), wxT("1"), wxT("2"), wxT("3") };
	int SegmentChoiceNChoices = sizeof( SegmentChoiceChoices ) / sizeof( wxString );
	SegmentChoice = new wxChoice( RenderStylesPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, SegmentChoiceNChoices, SegmentChoiceChoices, 0 );
	SegmentChoice->SetSelection( 0 );
	bSizer20111->Add( SegmentChoice, 0, wxALL, 5 );
	
	DensityDataLoadedFileText111 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataLoadedFileText111->Wrap( -1 );
	bSizer20111->Add( DensityDataLoadedFileText111, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxArrayString SegmentRenderStyleChoiceChoices;
	SegmentRenderStyleChoice = new wxChoice( RenderStylesPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, SegmentRenderStyleChoiceChoices, 0 );
	SegmentRenderStyleChoice->SetSelection( 0 );
	bSizer20111->Add( SegmentRenderStyleChoice, 0, wxALL, 5 );
	
	SegmentEnabledCheck = new wxCheckBox( RenderStylesPanel, wxID_ANY, wxT("Enabled"), wxDefaultPosition, wxDefaultSize, 0 );
	SegmentEnabledCheck->SetValue(true); 
	bSizer20111->Add( SegmentEnabledCheck, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	SegmentedDataNodeSizer->Add( bSizer20111, 1, wxEXPAND, 5 );
	
	
	RenderStylesSizer->Add( SegmentedDataNodeSizer, 0, wxEXPAND, 5 );
	
	IsoSurfacesNodeSizer = new wxStaticBoxSizer( new wxStaticBox( RenderStylesPanel, wxID_ANY, wxT("Iso surfaces") ), wxVERTICAL );
	
	wxBoxSizer* bSizer2012;
	bSizer2012 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText312 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("Iso values:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText312->Wrap( -1 );
	bSizer2012->Add( m_staticText312, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	IsoValuesText = new wxTextCtrl( RenderStylesPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	IsoValuesText->SetMaxLength( 0 ); 
	bSizer2012->Add( IsoValuesText, 0, wxALL, 5 );
	
	
	IsoSurfacesNodeSizer->Add( bSizer2012, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer201111;
	bSizer201111 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText31111 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("Surface:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText31111->Wrap( -1 );
	bSizer201111->Add( m_staticText31111, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString IsoSurfaceChoiceChoices[] = { wxT("0"), wxT("1"), wxT("2"), wxT("3"), wxT("4"), wxT("5"), wxT("6"), wxT("7"), wxT("8"), wxT("9") };
	int IsoSurfaceChoiceNChoices = sizeof( IsoSurfaceChoiceChoices ) / sizeof( wxString );
	IsoSurfaceChoice = new wxChoice( RenderStylesPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, IsoSurfaceChoiceNChoices, IsoSurfaceChoiceChoices, 0 );
	IsoSurfaceChoice->SetSelection( 0 );
	bSizer201111->Add( IsoSurfaceChoice, 0, wxALL, 5 );
	
	DensityDataLoadedFileText1111 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataLoadedFileText1111->Wrap( -1 );
	bSizer201111->Add( DensityDataLoadedFileText1111, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxArrayString IsoSurfaceRenderStyleChoiceChoices;
	IsoSurfaceRenderStyleChoice = new wxChoice( RenderStylesPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, IsoSurfaceRenderStyleChoiceChoices, 0 );
	IsoSurfaceRenderStyleChoice->SetSelection( 0 );
	bSizer201111->Add( IsoSurfaceRenderStyleChoice, 0, wxALL, 5 );
	
	
	IsoSurfacesNodeSizer->Add( bSizer201111, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer20121;
	bSizer20121 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText3121 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("Contour step size:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText3121->Wrap( -1 );
	bSizer20121->Add( m_staticText3121, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	ContourStepSizeText = new wxTextCtrl( RenderStylesPanel, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER );
	ContourStepSizeText->SetMaxLength( 0 ); 
	bSizer20121->Add( ContourStepSizeText, 0, wxALL, 5 );
	
	
	IsoSurfacesNodeSizer->Add( bSizer20121, 0, 0, 5 );
	
	wxBoxSizer* bSizer20122;
	bSizer20122 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText3122 = new wxStaticText( RenderStylesPanel, wxID_ANY, wxT("Surface tolerance:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText3122->Wrap( -1 );
	bSizer20122->Add( m_staticText3122, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	SurfaceToleranceText = new wxTextCtrl( RenderStylesPanel, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER );
	SurfaceToleranceText->SetMaxLength( 0 ); 
	bSizer20122->Add( SurfaceToleranceText, 0, wxALL, 5 );
	
	
	IsoSurfacesNodeSizer->Add( bSizer20122, 0, 0, 5 );
	
	
	RenderStylesSizer->Add( IsoSurfacesNodeSizer, 1, wxEXPAND, 5 );
	
	
	RenderStylesPanel->SetSizer( RenderStylesSizer );
	RenderStylesPanel->Layout();
	RenderStylesSizer->Fit( RenderStylesPanel );
	m_notebook2->AddPage( RenderStylesPanel, wxT("Render styles"), false );
	
	bSizer9->Add( m_notebook2, 1, wxEXPAND | wxALL, 5 );
	
	
	this->SetSizer( bSizer9 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( MainDialog::OnCloseDialog ) );
	this->Connect( wxEVT_IDLE, wxIdleEventHandler( MainDialog::OnIdle ) );
	LoadButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnLoadButton ), NULL, this );
	LoadRawButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnLoadRawButton ), NULL, this );
	SaveAsNrrdButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnSaveAsNrrdButton ), NULL, this );
	m_choice8->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnDataFilterChoice ), NULL, this );
	RendererChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnRendererChoice ), NULL, this );
	m_colourPicker19->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( MainDialog::OnBackgroundColorChanged ), NULL, this );
	RayStepText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnRayStep ), NULL, this );
	m_checkBox13->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnUseEmptySpaceSkipping ), NULL, this );
	m_checkBox15->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnShowNonEmptySpace ), NULL, this );
	m_choice7->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnEmptySpaceResolution ), NULL, this );
	m_checkBox14->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnStopRaysAtGeom ), NULL, this );
	m_checkBox16->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnUseStochasticJittering ), NULL, this );
	NrSlicesText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnNrSlices ), NULL, this );
	m_choice18->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnVolumeNodeChoice ), NULL, this );
	m_button101->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnShowStyleEditor ), NULL, this );
	m_button14->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnSaveAsX3DButton ), NULL, this );
	VolumeDataRenderStyleChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnRenderStyleChoice ), NULL, this );
	m_button9->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnLoadSegmentDataButton ), NULL, this );
	SegmentChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnSegmentChoice ), NULL, this );
	SegmentRenderStyleChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnSegmentStyleChoice ), NULL, this );
	SegmentEnabledCheck->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnSegmentEnabledCheck ), NULL, this );
	IsoValuesText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnIsoValueChange ), NULL, this );
	IsoSurfaceChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnIsoSurfaceChoice ), NULL, this );
	IsoSurfaceRenderStyleChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnIsoSurfaceStyleChoice ), NULL, this );
	ContourStepSizeText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnContourStepSizeChange ), NULL, this );
	SurfaceToleranceText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnSurfaceToleranceChange ), NULL, this );
}

MainDialog::~MainDialog()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( MainDialog::OnCloseDialog ) );
	this->Disconnect( wxEVT_IDLE, wxIdleEventHandler( MainDialog::OnIdle ) );
	LoadButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnLoadButton ), NULL, this );
	LoadRawButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnLoadRawButton ), NULL, this );
	SaveAsNrrdButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnSaveAsNrrdButton ), NULL, this );
	m_choice8->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnDataFilterChoice ), NULL, this );
	RendererChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnRendererChoice ), NULL, this );
	m_colourPicker19->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( MainDialog::OnBackgroundColorChanged ), NULL, this );
	RayStepText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnRayStep ), NULL, this );
	m_checkBox13->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnUseEmptySpaceSkipping ), NULL, this );
	m_checkBox15->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnShowNonEmptySpace ), NULL, this );
	m_choice7->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnEmptySpaceResolution ), NULL, this );
	m_checkBox14->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnStopRaysAtGeom ), NULL, this );
	m_checkBox16->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnUseStochasticJittering ), NULL, this );
	NrSlicesText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnNrSlices ), NULL, this );
	m_choice18->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnVolumeNodeChoice ), NULL, this );
	m_button101->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnShowStyleEditor ), NULL, this );
	m_button14->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnSaveAsX3DButton ), NULL, this );
	VolumeDataRenderStyleChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnRenderStyleChoice ), NULL, this );
	m_button9->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( MainDialog::OnLoadSegmentDataButton ), NULL, this );
	SegmentChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnSegmentChoice ), NULL, this );
	SegmentRenderStyleChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnSegmentStyleChoice ), NULL, this );
	SegmentEnabledCheck->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( MainDialog::OnSegmentEnabledCheck ), NULL, this );
	IsoValuesText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnIsoValueChange ), NULL, this );
	IsoSurfaceChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnIsoSurfaceChoice ), NULL, this );
	IsoSurfaceRenderStyleChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MainDialog::OnIsoSurfaceStyleChoice ), NULL, this );
	ContourStepSizeText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnContourStepSizeChange ), NULL, this );
	SurfaceToleranceText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( MainDialog::OnSurfaceToleranceChange ), NULL, this );
	
}

StyleDialog::StyleDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	StyleDialogMainSizer = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer20;
	sbSizer20 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Available styles") ), wxVERTICAL );
	
	wxBoxSizer* bSizer31;
	bSizer31 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText2222 = new wxStaticText( this, wxID_ANY, wxT("Style name:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2222->Wrap( -1 );
	bSizer31->Add( m_staticText2222, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxArrayString StyleNameChoiceChoices;
	StyleNameChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, StyleNameChoiceChoices, 0 );
	StyleNameChoice->SetSelection( 0 );
	bSizer31->Add( StyleNameChoice, 0, wxALL, 5 );
	
	m_button7 = new wxButton( this, wxID_ANY, wxT("New"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer31->Add( m_button7, 0, wxALL, 5 );
	
	m_button8 = new wxButton( this, wxID_ANY, wxT("Delete"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer31->Add( m_button8, 0, wxALL, 5 );
	
	SaveStyleButton = new wxButton( this, wxID_ANY, wxT("Save"), wxDefaultPosition, wxDefaultSize, 0 );
	SaveStyleButton->Enable( false );
	
	bSizer31->Add( SaveStyleButton, 0, wxALL, 5 );
	
	
	sbSizer20->Add( bSizer31, 0, 0, 5 );
	
	wxBoxSizer* bSizer20211;
	bSizer20211 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText222121 = new wxStaticText( this, wxID_ANY, wxT("Style type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText222121->Wrap( -1 );
	bSizer20211->Add( m_staticText222121, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString StyleTypeChoiceChoices[] = { wxT("None"), wxT("OpacityMapVolumeStyle"), wxT("BlendedVolumeStyle"), wxT("BoundaryEnhancementVolumeStyle"), wxT("CartoonVolumeStyle"), wxT("ComposedVolumeStyle"), wxT("EdgeEnhancementVolumeStyle"), wxT("ProjectionVolumeStyle"), wxT("ShadedVolumeStyle"), wxT("SilhouetteEnhancementVolumeStyle"), wxT("ToneMappedVolumeStyle") };
	int StyleTypeChoiceNChoices = sizeof( StyleTypeChoiceChoices ) / sizeof( wxString );
	StyleTypeChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, StyleTypeChoiceNChoices, StyleTypeChoiceChoices, 0 );
	StyleTypeChoice->SetSelection( 0 );
	bSizer20211->Add( StyleTypeChoice, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	sbSizer20->Add( bSizer20211, 0, 0, 5 );
	
	
	StyleDialogMainSizer->Add( sbSizer20, 0, wxEXPAND, 5 );
	
	
	this->SetSizer( StyleDialogMainSizer );
	this->Layout();
	
	// Connect Events
	StyleNameChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( StyleDialog::OnRenderStyleChoice ), NULL, this );
	m_button7->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( StyleDialog::OnNewStyleButton ), NULL, this );
	m_button8->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( StyleDialog::OnDeleteStyleButton ), NULL, this );
	SaveStyleButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( StyleDialog::OnSaveStyleButton ), NULL, this );
	StyleTypeChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( StyleDialog::OnStyleTypeChoice ), NULL, this );
}

StyleDialog::~StyleDialog()
{
	// Disconnect Events
	StyleNameChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( StyleDialog::OnRenderStyleChoice ), NULL, this );
	m_button7->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( StyleDialog::OnNewStyleButton ), NULL, this );
	m_button8->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( StyleDialog::OnDeleteStyleButton ), NULL, this );
	SaveStyleButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( StyleDialog::OnSaveStyleButton ), NULL, this );
	StyleTypeChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( StyleDialog::OnStyleTypeChoice ), NULL, this );
	
}

ShadedVolumeStyleOptions::ShadedVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	ShadedVolumeStyleOptions1 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("ShadedVolumeStyle") ), wxVERTICAL );
	
	wxBoxSizer* bSizer491;
	bSizer491 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer62;
	bSizer62 = new wxBoxSizer( wxHORIZONTAL );
	
	ShadedOnMaterialCheck = new wxCheckBox( this, wxID_ANY, wxT("Use Material"), wxDefaultPosition, wxDefaultSize, 0 );
	ShadedOnMaterialCheck->SetValue(true); 
	ShadedOnMaterialCheck->SetToolTip( wxT("Decide if using Material node to specify color. If not, the color from previous style is used.") );
	
	bSizer62->Add( ShadedOnMaterialCheck, 0, wxALL, 5 );
	
	
	bSizer491->Add( bSizer62, 0, 0, 5 );
	
	ShadedMaterialWidgetsSizer = new wxFlexGridSizer( 1, 2, 0, 0 );
	ShadedMaterialWidgetsSizer->SetFlexibleDirection( wxBOTH );
	ShadedMaterialWidgetsSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	wxGridSizer* gSizer11;
	gSizer11 = new wxGridSizer( 3, 2, 0, 0 );
	
	DiffuseColorText = new wxStaticText( this, wxID_ANY, wxT("Diffuse color:"), wxDefaultPosition, wxDefaultSize, 0 );
	DiffuseColorText->Wrap( -1 );
	gSizer11->Add( DiffuseColorText, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	DiffuseColorPicker = new wxColourPickerCtrl( this, wxID_ANY, wxColour( 204, 204, 204 ), wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	gSizer11->Add( DiffuseColorPicker, 0, wxALL, 5 );
	
	EmissiveColorText = new wxStaticText( this, wxID_ANY, wxT("Emissive color:"), wxDefaultPosition, wxDefaultSize, 0 );
	EmissiveColorText->Wrap( -1 );
	gSizer11->Add( EmissiveColorText, 0, wxALL, 5 );
	
	EmissiveColorPicker = new wxColourPickerCtrl( this, wxID_ANY, *wxBLACK, wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	gSizer11->Add( EmissiveColorPicker, 0, wxALL, 5 );
	
	SpecularColorText = new wxStaticText( this, wxID_ANY, wxT("Specular color:"), wxDefaultPosition, wxDefaultSize, 0 );
	SpecularColorText->Wrap( -1 );
	gSizer11->Add( SpecularColorText, 0, wxALL, 5 );
	
	SpecularColorPicker = new wxColourPickerCtrl( this, wxID_ANY, *wxBLACK, wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	gSizer11->Add( SpecularColorPicker, 0, wxALL, 5 );
	
	
	ShadedMaterialWidgetsSizer->Add( gSizer11, 0, 0, 5 );
	
	wxFlexGridSizer* fgSizer10;
	fgSizer10 = new wxFlexGridSizer( 2, 3, 0, 0 );
	fgSizer10->SetFlexibleDirection( wxBOTH );
	fgSizer10->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	ShadedTransparencyLabel = new wxStaticText( this, wxID_ANY, wxT("Transparency:"), wxDefaultPosition, wxDefaultSize, 0 );
	ShadedTransparencyLabel->Wrap( -1 );
	fgSizer10->Add( ShadedTransparencyLabel, 0, wxALL, 5 );
	
	ShadedTransparencySlider = new wxSlider( this, wxID_ANY, 0, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer10->Add( ShadedTransparencySlider, 0, wxALL, 5 );
	
	ShadedTransparencyText = new wxTextCtrl( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	ShadedTransparencyText->SetMaxLength( 0 ); 
	fgSizer10->Add( ShadedTransparencyText, 0, wxALL, 5 );
	
	ShadedAmbientIntensityLabel = new wxStaticText( this, wxID_ANY, wxT("Ambient intensity:"), wxDefaultPosition, wxDefaultSize, 0 );
	ShadedAmbientIntensityLabel->Wrap( -1 );
	fgSizer10->Add( ShadedAmbientIntensityLabel, 0, wxALL, 5 );
	
	ShadedAmbientIntensitySlider = new wxSlider( this, wxID_ANY, 20, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer10->Add( ShadedAmbientIntensitySlider, 0, wxALL, 5 );
	
	ShadedAmbientIntensityText = new wxTextCtrl( this, wxID_ANY, wxT("0.2"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	ShadedAmbientIntensityText->SetMaxLength( 0 ); 
	fgSizer10->Add( ShadedAmbientIntensityText, 0, wxALL, 5 );
	
	
	ShadedMaterialWidgetsSizer->Add( fgSizer10, 1, wxEXPAND, 5 );
	
	
	bSizer491->Add( ShadedMaterialWidgetsSizer, 0, 0, 5 );
	
	wxBoxSizer* bSizer54;
	bSizer54 = new wxBoxSizer( wxHORIZONTAL );
	
	ShadedLightingBox = new wxCheckBox( this, wxID_ANY, wxT("Lighting"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer54->Add( ShadedLightingBox, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	ShadedShadowsBox = new wxCheckBox( this, wxID_ANY, wxT("Shadows"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer54->Add( ShadedShadowsBox, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText611 = new wxStaticText( this, wxID_ANY, wxT("Phase function:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText611->Wrap( -1 );
	bSizer54->Add( m_staticText611, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString m_choice51Choices[] = { wxT("None") };
	int m_choice51NChoices = sizeof( m_choice51Choices ) / sizeof( wxString );
	m_choice51 = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_choice51NChoices, m_choice51Choices, 0 );
	m_choice51->SetSelection( 0 );
	bSizer54->Add( m_choice51, 0, wxALL, 5 );
	
	
	bSizer491->Add( bSizer54, 0, 0, 5 );
	
	
	ShadedVolumeStyleOptions1->Add( bSizer491, 1, wxEXPAND, 5 );
	
	
	this->SetSizer( ShadedVolumeStyleOptions1 );
	this->Layout();
	
	// Connect Events
	ShadedOnMaterialCheck->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ShadedVolumeStyleOptions::OnShadedUseMaterialCheck ), NULL, this );
	DiffuseColorPicker->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ShadedVolumeStyleOptions::OnShadedDiffuseColorChanged ), NULL, this );
	EmissiveColorPicker->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ShadedVolumeStyleOptions::OnShadedEmissiveColorChanged ), NULL, this );
	SpecularColorPicker->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ShadedVolumeStyleOptions::OnShadedSpecularColorChanged ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedLightingBox->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ShadedVolumeStyleOptions::OnShadedLightingBox ), NULL, this );
	ShadedShadowsBox->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ShadedVolumeStyleOptions::OnShadedShadowsBox ), NULL, this );
}

ShadedVolumeStyleOptions::~ShadedVolumeStyleOptions()
{
	// Disconnect Events
	ShadedOnMaterialCheck->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ShadedVolumeStyleOptions::OnShadedUseMaterialCheck ), NULL, this );
	DiffuseColorPicker->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ShadedVolumeStyleOptions::OnShadedDiffuseColorChanged ), NULL, this );
	EmissiveColorPicker->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ShadedVolumeStyleOptions::OnShadedEmissiveColorChanged ), NULL, this );
	SpecularColorPicker->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ShadedVolumeStyleOptions::OnShadedSpecularColorChanged ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedTransparencySlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedTransparencyScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedAmbientIntensitySlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( ShadedVolumeStyleOptions::OnShadedAmbientIntensityScroll ), NULL, this );
	ShadedLightingBox->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ShadedVolumeStyleOptions::OnShadedLightingBox ), NULL, this );
	ShadedShadowsBox->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( ShadedVolumeStyleOptions::OnShadedShadowsBox ), NULL, this );
	
}

OpacityMapVolumeStyleOptions::OpacityMapVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	OpacityMapStyleOptions = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("OpacityMapStyle") ), wxVERTICAL );
	
	wxBoxSizer* bSizer49;
	bSizer49 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer48;
	bSizer48 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText61 = new wxStaticText( this, wxID_ANY, wxT("Type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText61->Wrap( -1 );
	bSizer48->Add( m_staticText61, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString TypeChoiceChoices[] = { wxT("simple"), wxT("preintegrated_fast"), wxT("preintegrated") };
	int TypeChoiceNChoices = sizeof( TypeChoiceChoices ) / sizeof( wxString );
	TypeChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, TypeChoiceNChoices, TypeChoiceChoices, 0 );
	TypeChoice->SetSelection( 0 );
	bSizer48->Add( TypeChoice, 0, wxALL, 5 );
	
	
	bSizer49->Add( bSizer48, 0, 0, 5 );
	
	wxBoxSizer* bSizer72;
	bSizer72 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText77 = new wxStaticText( this, wxID_ANY, wxT("Transfer function:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText77->Wrap( -1 );
	bSizer72->Add( m_staticText77, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString TransferFunctionChoiceChoices[] = { wxT("File"), wxT("Window") };
	int TransferFunctionChoiceNChoices = sizeof( TransferFunctionChoiceChoices ) / sizeof( wxString );
	TransferFunctionChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, TransferFunctionChoiceNChoices, TransferFunctionChoiceChoices, 0 );
	TransferFunctionChoice->SetSelection( 1 );
	bSizer72->Add( TransferFunctionChoice, 0, wxALL, 5 );
	
	
	bSizer49->Add( bSizer72, 0, 0, 5 );
	
	FileOptionsSizer = new wxBoxSizer( wxHORIZONTAL );
	
	m_button20 = new wxButton( this, wxID_ANY, wxT("Load image.."), wxDefaultPosition, wxDefaultSize, 0 );
	m_button20->Enable( false );
	
	FileOptionsSizer->Add( m_button20, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText78 = new wxStaticText( this, wxID_ANY, wxT("Loaded file:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText78->Wrap( -1 );
	m_staticText78->Enable( false );
	
	FileOptionsSizer->Add( m_staticText78, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	FilenameText = new wxStaticText( this, wxID_ANY, wxT("None"), wxDefaultPosition, wxDefaultSize, 0 );
	FilenameText->Wrap( -1 );
	FilenameText->Enable( false );
	
	FileOptionsSizer->Add( FilenameText, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer49->Add( FileOptionsSizer, 0, 0, 5 );
	
	WindowOptionsSizer = new wxFlexGridSizer( 2, 3, 0, 0 );
	WindowOptionsSizer->SetFlexibleDirection( wxBOTH );
	WindowOptionsSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText80 = new wxStaticText( this, wxID_ANY, wxT("Window center:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText80->Wrap( -1 );
	WindowOptionsSizer->Add( m_staticText80, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	WindowCenterSlider = new wxSlider( this, wxID_ANY, 50, 0, 100, wxDefaultPosition, wxSize( 150,-1 ), wxSL_HORIZONTAL );
	WindowOptionsSizer->Add( WindowCenterSlider, 0, wxALL, 5 );
	
	WindowCenterText = new wxStaticText( this, wxID_ANY, wxT("0.5"), wxDefaultPosition, wxDefaultSize, 0 );
	WindowCenterText->Wrap( -1 );
	WindowOptionsSizer->Add( WindowCenterText, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText801 = new wxStaticText( this, wxID_ANY, wxT("Window width:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText801->Wrap( -1 );
	WindowOptionsSizer->Add( m_staticText801, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	WindowWidthSlider = new wxSlider( this, wxID_ANY, 100, 1, 100, wxDefaultPosition, wxSize( 150,-1 ), wxSL_HORIZONTAL );
	WindowOptionsSizer->Add( WindowWidthSlider, 0, wxALL, 5 );
	
	WindowWidthText = new wxStaticText( this, wxID_ANY, wxT("1"), wxDefaultPosition, wxDefaultSize, 0 );
	WindowWidthText->Wrap( -1 );
	WindowOptionsSizer->Add( WindowWidthText, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer49->Add( WindowOptionsSizer, 1, wxEXPAND, 5 );
	
	
	OpacityMapStyleOptions->Add( bSizer49, 1, wxEXPAND, 5 );
	
	
	this->SetSizer( OpacityMapStyleOptions );
	this->Layout();
	
	// Connect Events
	TypeChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( OpacityMapVolumeStyleOptions::OnTypeChoice ), NULL, this );
	TransferFunctionChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( OpacityMapVolumeStyleOptions::OnTransferFunctionChoice ), NULL, this );
	m_button20->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( OpacityMapVolumeStyleOptions::OnLoadImageButton ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
}

OpacityMapVolumeStyleOptions::~OpacityMapVolumeStyleOptions()
{
	// Disconnect Events
	TypeChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( OpacityMapVolumeStyleOptions::OnTypeChoice ), NULL, this );
	TransferFunctionChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( OpacityMapVolumeStyleOptions::OnTransferFunctionChoice ), NULL, this );
	m_button20->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( OpacityMapVolumeStyleOptions::OnLoadImageButton ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowCenterSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowCenterScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	WindowWidthSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( OpacityMapVolumeStyleOptions::OnWindowWidthScroll ), NULL, this );
	
}

ToneMappedVolumeStyleOptions::ToneMappedVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer36;
	bSizer36 = new wxBoxSizer( wxHORIZONTAL );
	
	wxStaticBoxSizer* ToneMappedStyle;
	ToneMappedStyle = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("ToneMappedStyle") ), wxVERTICAL );
	
	wxBoxSizer* bSizer42;
	bSizer42 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer14;
	fgSizer14 = new wxFlexGridSizer( 0, 2, 0, 0 );
	fgSizer14->SetFlexibleDirection( wxBOTH );
	fgSizer14->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	wxGridSizer* gSizer5;
	gSizer5 = new wxGridSizer( 2, 2, 0, 0 );
	
	m_staticText35 = new wxStaticText( this, wxID_ANY, wxT("Cool color:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText35->Wrap( -1 );
	gSizer5->Add( m_staticText35, 0, wxALL, 5 );
	
	CoolColorPicker = new wxColourPickerCtrl( this, wxID_ANY, wxColour( 0, 0, 255 ), wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	gSizer5->Add( CoolColorPicker, 0, wxALL, 5 );
	
	m_staticText36 = new wxStaticText( this, wxID_ANY, wxT("Warm color:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText36->Wrap( -1 );
	gSizer5->Add( m_staticText36, 0, wxALL, 5 );
	
	WarmColorPicker = new wxColourPickerCtrl( this, wxID_ANY, wxColour( 255, 255, 0 ), wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	gSizer5->Add( WarmColorPicker, 0, wxALL, 5 );
	
	
	fgSizer14->Add( gSizer5, 0, 0, 5 );
	
	wxFlexGridSizer* fgSizer10;
	fgSizer10 = new wxFlexGridSizer( 2, 3, 0, 0 );
	fgSizer10->SetFlexibleDirection( wxBOTH );
	fgSizer10->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText86 = new wxStaticText( this, wxID_ANY, wxT("Transparency"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText86->Wrap( -1 );
	fgSizer10->Add( m_staticText86, 0, wxALL, 5 );
	
	ToneMappedCoolColorTransparencySlider = new wxSlider( this, wxID_ANY, 0, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer10->Add( ToneMappedCoolColorTransparencySlider, 0, wxALL, 5 );
	
	ToneMappedCoolColorTransparencyText = new wxTextCtrl( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	ToneMappedCoolColorTransparencyText->SetMaxLength( 0 ); 
	fgSizer10->Add( ToneMappedCoolColorTransparencyText, 0, wxALL, 5 );
	
	m_staticText861 = new wxStaticText( this, wxID_ANY, wxT("Transparency"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText861->Wrap( -1 );
	fgSizer10->Add( m_staticText861, 0, wxALL, 5 );
	
	ToneMappedWarmColorTransparencySlider = new wxSlider( this, wxID_ANY, 0, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer10->Add( ToneMappedWarmColorTransparencySlider, 0, wxALL, 5 );
	
	ToneMappedWarmColorTransparencyText = new wxTextCtrl( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	ToneMappedWarmColorTransparencyText->SetMaxLength( 0 ); 
	fgSizer10->Add( ToneMappedWarmColorTransparencyText, 0, wxALL, 5 );
	
	
	fgSizer14->Add( fgSizer10, 1, wxEXPAND, 5 );
	
	
	bSizer42->Add( fgSizer14, 1, 0, 5 );
	
	
	ToneMappedStyle->Add( bSizer42, 0, wxEXPAND, 5 );
	
	
	bSizer36->Add( ToneMappedStyle, 0, 0, 5 );
	
	
	this->SetSizer( bSizer36 );
	this->Layout();
	
	// Connect Events
	CoolColorPicker->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorChanged ), NULL, this );
	WarmColorPicker->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorChanged ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
}

ToneMappedVolumeStyleOptions::~ToneMappedVolumeStyleOptions()
{
	// Disconnect Events
	CoolColorPicker->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorChanged ), NULL, this );
	WarmColorPicker->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorChanged ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedCoolColorTransparencySlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnCoolColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	ToneMappedWarmColorTransparencySlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( ToneMappedVolumeStyleOptions::OnWarmColorTransparencyScroll ), NULL, this );
	
}

CartoonVolumeStyleOptions::CartoonVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxStaticBoxSizer* sbSizer12;
	sbSizer12 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("CartoonStyle") ), wxVERTICAL );
	
	wxBoxSizer* bSizer42;
	bSizer42 = new wxBoxSizer( wxHORIZONTAL );
	
	wxGridSizer* gSizer6;
	gSizer6 = new wxGridSizer( 0, 2, 0, 0 );
	
	m_staticText35 = new wxStaticText( this, wxID_ANY, wxT("Parallel color:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText35->Wrap( -1 );
	gSizer6->Add( m_staticText35, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	ParallelColorPicker = new wxColourPickerCtrl( this, wxID_ANY, wxColour( 0, 0, 0 ), wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	gSizer6->Add( ParallelColorPicker, 0, wxALL, 5 );
	
	m_staticText36 = new wxStaticText( this, wxID_ANY, wxT("Orthogonal color"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText36->Wrap( -1 );
	gSizer6->Add( m_staticText36, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	OrthogonalColorPicker = new wxColourPickerCtrl( this, wxID_ANY, wxColour( 255, 255, 255 ), wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	gSizer6->Add( OrthogonalColorPicker, 0, wxALL, 5 );
	
	
	bSizer42->Add( gSizer6, 1, wxEXPAND, 5 );
	
	wxFlexGridSizer* fgSizer10;
	fgSizer10 = new wxFlexGridSizer( 2, 3, 0, 0 );
	fgSizer10->SetFlexibleDirection( wxBOTH );
	fgSizer10->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText86 = new wxStaticText( this, wxID_ANY, wxT("Transparency"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText86->Wrap( -1 );
	fgSizer10->Add( m_staticText86, 0, wxALL, 5 );
	
	CartoonParallelColorTransparencySlider = new wxSlider( this, wxID_ANY, 100, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer10->Add( CartoonParallelColorTransparencySlider, 0, wxALL, 5 );
	
	CartoonParallelColorTransparencyText = new wxTextCtrl( this, wxID_ANY, wxT("1.0"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	CartoonParallelColorTransparencyText->SetMaxLength( 0 ); 
	fgSizer10->Add( CartoonParallelColorTransparencyText, 0, wxALL, 5 );
	
	m_staticText861 = new wxStaticText( this, wxID_ANY, wxT("Transparency"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText861->Wrap( -1 );
	fgSizer10->Add( m_staticText861, 0, wxALL, 5 );
	
	CartoonOrthogonalColorTransparencySlider = new wxSlider( this, wxID_ANY, 100, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	fgSizer10->Add( CartoonOrthogonalColorTransparencySlider, 0, wxALL, 5 );
	
	CartoonOrthogonalColorTransparencyText = new wxTextCtrl( this, wxID_ANY, wxT("1.0"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	CartoonOrthogonalColorTransparencyText->SetMaxLength( 0 ); 
	fgSizer10->Add( CartoonOrthogonalColorTransparencyText, 0, wxALL, 5 );
	
	
	bSizer42->Add( fgSizer10, 1, wxEXPAND, 5 );
	
	
	sbSizer12->Add( bSizer42, 0, 0, 5 );
	
	wxBoxSizer* bSizer46;
	bSizer46 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText721 = new wxStaticText( this, wxID_ANY, wxT("Steps:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText721->Wrap( -1 );
	bSizer46->Add( m_staticText721, 0, wxALL, 5 );
	
	NrStepsSlider = new wxSlider( this, wxID_ANY, 4, 1, 64, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	bSizer46->Add( NrStepsSlider, 0, wxALL, 5 );
	
	NrStepsText = new wxTextCtrl( this, wxID_ANY, wxT("1"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	NrStepsText->SetMaxLength( 0 ); 
	bSizer46->Add( NrStepsText, 0, wxALL, 5 );
	
	
	sbSizer12->Add( bSizer46, 0, 0, 5 );
	
	
	this->SetSizer( sbSizer12 );
	this->Layout();
	
	// Connect Events
	ParallelColorPicker->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( CartoonVolumeStyleOptions::OnParallelColorChange ), NULL, this );
	OrthogonalColorPicker->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorChange ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
}

CartoonVolumeStyleOptions::~CartoonVolumeStyleOptions()
{
	// Disconnect Events
	ParallelColorPicker->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( CartoonVolumeStyleOptions::OnParallelColorChange ), NULL, this );
	OrthogonalColorPicker->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorChange ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonParallelColorTransparencySlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CartoonVolumeStyleOptions::OnParallelColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	CartoonOrthogonalColorTransparencySlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CartoonVolumeStyleOptions::OnOrthogonalColorTransparencyScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	NrStepsSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CartoonVolumeStyleOptions::OnStepsScroll ), NULL, this );
	
}

MIPVolumeStyleOptions::MIPVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxStaticBoxSizer* sbSizer13;
	sbSizer13 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("ProjectionStyle") ), wxHORIZONTAL );
	
	wxBoxSizer* bSizer57;
	bSizer57 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText74 = new wxStaticText( this, wxID_ANY, wxT("Projection type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText74->Wrap( -1 );
	bSizer57->Add( m_staticText74, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString ProjectionStyleTypeChoiceChoices[] = { wxT("MAX"), wxT("MIN"), wxT("AVERAGE") };
	int ProjectionStyleTypeChoiceNChoices = sizeof( ProjectionStyleTypeChoiceChoices ) / sizeof( wxString );
	ProjectionStyleTypeChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, ProjectionStyleTypeChoiceNChoices, ProjectionStyleTypeChoiceChoices, 0 );
	ProjectionStyleTypeChoice->SetSelection( 0 );
	bSizer57->Add( ProjectionStyleTypeChoice, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	sbSizer13->Add( bSizer57, 0, 0, 5 );
	
	wxBoxSizer* bSizer47;
	bSizer47 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText48 = new wxStaticText( this, wxID_ANY, wxT("Intensity threshold:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText48->Wrap( -1 );
	bSizer47->Add( m_staticText48, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	IntensityThresholdSlider = new wxSlider( this, wxID_ANY, 0, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	bSizer47->Add( IntensityThresholdSlider, 0, wxALL, 5 );
	
	IntensityThresholdText = new wxTextCtrl( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 50,-1 ), wxTE_READONLY );
	IntensityThresholdText->SetMaxLength( 0 ); 
	bSizer47->Add( IntensityThresholdText, 0, wxALL, 5 );
	
	
	sbSizer13->Add( bSizer47, 0, 0, 5 );
	
	
	this->SetSizer( sbSizer13 );
	this->Layout();
	
	// Connect Events
	ProjectionStyleTypeChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MIPVolumeStyleOptions::OnProjectionStyleTypeChoice ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
}

MIPVolumeStyleOptions::~MIPVolumeStyleOptions()
{
	// Disconnect Events
	ProjectionStyleTypeChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( MIPVolumeStyleOptions::OnProjectionStyleTypeChoice ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	IntensityThresholdSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( MIPVolumeStyleOptions::OnIntensityThresholdSlider ), NULL, this );
	
}

ComposedVolumeStyleOptionsPart::ComposedVolumeStyleOptionsPart( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	FlexSizer = new wxFlexGridSizer( 1, 2, 0, 0 );
	FlexSizer->AddGrowableCol( 0 );
	FlexSizer->SetFlexibleDirection( wxBOTH );
	FlexSizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	OptionsSizer = new wxBoxSizer( wxVERTICAL );
	
	m_panel7 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	OptionsSizer->Add( m_panel7, 1, wxEXPAND | wxALL, 5 );
	
	
	FlexSizer->Add( OptionsSizer, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer63;
	bSizer63 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer62;
	bSizer62 = new wxBoxSizer( wxHORIZONTAL );
	
	m_button13 = new wxButton( this, wxID_ANY, wxT("+"), wxDefaultPosition, wxSize( 20,20 ), 0 );
	bSizer62->Add( m_button13, 0, wxALL, 5 );
	
	m_button14 = new wxButton( this, wxID_ANY, wxT("-"), wxDefaultPosition, wxSize( 20,20 ), 0 );
	bSizer62->Add( m_button14, 0, wxALL, 5 );
	
	
	bSizer63->Add( bSizer62, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer631;
	bSizer631 = new wxBoxSizer( wxHORIZONTAL );
	
	m_button15 = new wxButton( this, wxID_ANY, wxT("<"), wxDefaultPosition, wxSize( 20,20 ), 0 );
	m_button15->Hide();
	
	bSizer631->Add( m_button15, 0, wxALL, 5 );
	
	m_button16 = new wxButton( this, wxID_ANY, wxT(">"), wxDefaultPosition, wxSize( 20,20 ), 0 );
	m_button16->Hide();
	
	bSizer631->Add( m_button16, 0, wxALL, 5 );
	
	
	bSizer63->Add( bSizer631, 1, wxEXPAND, 5 );
	
	
	FlexSizer->Add( bSizer63, 1, wxEXPAND|wxALIGN_RIGHT, 5 );
	
	
	this->SetSizer( FlexSizer );
	this->Layout();
	
	// Connect Events
	m_button13->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ComposedVolumeStyleOptionsPart::OnAddStyleButton ), NULL, this );
	m_button14->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ComposedVolumeStyleOptionsPart::OnRemoveStyleButton ), NULL, this );
	m_button15->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ComposedVolumeStyleOptionsPart::OnMoveStyleUpButton ), NULL, this );
	m_button16->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ComposedVolumeStyleOptionsPart::OnMoveStyleDownButton ), NULL, this );
}

ComposedVolumeStyleOptionsPart::~ComposedVolumeStyleOptionsPart()
{
	// Disconnect Events
	m_button13->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ComposedVolumeStyleOptionsPart::OnAddStyleButton ), NULL, this );
	m_button14->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ComposedVolumeStyleOptionsPart::OnRemoveStyleButton ), NULL, this );
	m_button15->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ComposedVolumeStyleOptionsPart::OnMoveStyleUpButton ), NULL, this );
	m_button16->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( ComposedVolumeStyleOptionsPart::OnMoveStyleDownButton ), NULL, this );
	
}

ComposedVolumeStyleOptions::ComposedVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	StylePartsSizer = new wxBoxSizer( wxVERTICAL );
	
	
	this->SetSizer( StylePartsSizer );
	this->Layout();
}

ComposedVolumeStyleOptions::~ComposedVolumeStyleOptions()
{
}

SilhouetteEnhancementVolumeStyleOptions::SilhouetteEnhancementVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxStaticBoxSizer* sbSizer14;
	sbSizer14 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("SilhouetteEnhancementStyle") ), wxVERTICAL );
	
	wxBoxSizer* bSizer48;
	bSizer48 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer4;
	fgSizer4 = new wxFlexGridSizer( 3, 2, 0, 0 );
	fgSizer4->SetFlexibleDirection( wxBOTH );
	fgSizer4->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText48 = new wxStaticText( this, wxID_ANY, wxT("Silhouette boundary opacity:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText48->Wrap( -1 );
	fgSizer4->Add( m_staticText48, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	BoundaryOpacityText = new wxTextCtrl( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	BoundaryOpacityText->SetMaxLength( 0 ); 
	fgSizer4->Add( BoundaryOpacityText, 0, wxALL, 5 );
	
	m_staticText481 = new wxStaticText( this, wxID_ANY, wxT("Silhouette sharpness:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText481->Wrap( -1 );
	fgSizer4->Add( m_staticText481, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	SharpnessText = new wxTextCtrl( this, wxID_ANY, wxT("1"), wxDefaultPosition, wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	SharpnessText->SetMaxLength( 0 ); 
	fgSizer4->Add( SharpnessText, 0, wxALL, 5 );
	
	m_staticText721 = new wxStaticText( this, wxID_ANY, wxT("Silhouette retained opacity:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText721->Wrap( -1 );
	fgSizer4->Add( m_staticText721, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer46;
	bSizer46 = new wxBoxSizer( wxHORIZONTAL );
	
	RetainedOpacitySlider = new wxSlider( this, wxID_ANY, 50, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	bSizer46->Add( RetainedOpacitySlider, 0, wxALL, 5 );
	
	RetainedOpacityText = new wxTextCtrl( this, wxID_ANY, wxT("0.5"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	RetainedOpacityText->SetMaxLength( 0 ); 
	bSizer46->Add( RetainedOpacityText, 0, wxALL, 5 );
	
	
	fgSizer4->Add( bSizer46, 0, 0, 5 );
	
	
	bSizer48->Add( fgSizer4, 0, 0, 5 );
	
	
	sbSizer14->Add( bSizer48, 0, 0, 5 );
	
	
	this->SetSizer( sbSizer14 );
	this->Layout();
	
	// Connect Events
	BoundaryOpacityText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnBoundaryOpacityChanged ), NULL, this );
	SharpnessText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnSharpnessChanged ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
}

SilhouetteEnhancementVolumeStyleOptions::~SilhouetteEnhancementVolumeStyleOptions()
{
	// Disconnect Events
	BoundaryOpacityText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnBoundaryOpacityChanged ), NULL, this );
	SharpnessText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnSharpnessChanged ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( SilhouetteEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	
}

EdgeEnhancementVolumeStyleOptions::EdgeEnhancementVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxStaticBoxSizer* EdgeEnhancementStyle;
	EdgeEnhancementStyle = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("EdgeEnhancementStyle") ), wxVERTICAL );
	
	wxBoxSizer* bSizer38;
	bSizer38 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer42;
	bSizer42 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText35 = new wxStaticText( this, wxID_ANY, wxT("Edge color:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText35->Wrap( -1 );
	bSizer42->Add( m_staticText35, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	EdgeColorPicker = new wxColourPickerCtrl( this, wxID_ANY, wxColour( 0, 0, 0 ), wxDefaultPosition, wxDefaultSize, wxCLRP_DEFAULT_STYLE );
	bSizer42->Add( EdgeColorPicker, 0, wxALL, 5 );
	
	
	bSizer38->Add( bSizer42, 0, 0, 5 );
	
	wxBoxSizer* bSizer46;
	bSizer46 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText46 = new wxStaticText( this, wxID_ANY, wxT("Gradient threshold:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText46->Wrap( -1 );
	bSizer46->Add( m_staticText46, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	GradientThresholdSlider = new wxSlider( this, wxID_ANY, 25, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	bSizer46->Add( GradientThresholdSlider, 0, wxALL, 5 );
	
	GradientThresholdText = new wxTextCtrl( this, wxID_ANY, wxT("0.4"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	GradientThresholdText->SetMaxLength( 0 ); 
	bSizer46->Add( GradientThresholdText, 0, wxALL, 5 );
	
	
	bSizer38->Add( bSizer46, 0, 0, 5 );
	
	
	EdgeEnhancementStyle->Add( bSizer38, 1, wxEXPAND, 5 );
	
	
	this->SetSizer( EdgeEnhancementStyle );
	this->Layout();
	
	// Connect Events
	EdgeColorPicker->Connect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( EdgeEnhancementVolumeStyleOptions::OnEdgeColorChanged ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
}

EdgeEnhancementVolumeStyleOptions::~EdgeEnhancementVolumeStyleOptions()
{
	// Disconnect Events
	EdgeColorPicker->Disconnect( wxEVT_COMMAND_COLOURPICKER_CHANGED, wxColourPickerEventHandler( EdgeEnhancementVolumeStyleOptions::OnEdgeColorChanged ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	GradientThresholdSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( EdgeEnhancementVolumeStyleOptions::OnGradientThresholdScroll ), NULL, this );
	
}

BoundaryEnhancementVolumeStyleOptions::BoundaryEnhancementVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxStaticBoxSizer* BoundaryEnhancementStyle;
	BoundaryEnhancementStyle = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("BoundaryEnhancementStyle") ), wxVERTICAL );
	
	wxBoxSizer* bSizer48;
	bSizer48 = new wxBoxSizer( wxVERTICAL );
	
	wxFlexGridSizer* fgSizer4;
	fgSizer4 = new wxFlexGridSizer( 3, 2, 0, 0 );
	fgSizer4->SetFlexibleDirection( wxBOTH );
	fgSizer4->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText48 = new wxStaticText( this, wxID_ANY, wxT("Boundary opacity:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText48->Wrap( -1 );
	fgSizer4->Add( m_staticText48, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	BoundaryOpacityText = new wxTextCtrl( this, wxID_ANY, wxT("0"), wxDefaultPosition, wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	BoundaryOpacityText->SetMaxLength( 0 ); 
	fgSizer4->Add( BoundaryOpacityText, 0, wxALL, 5 );
	
	m_staticText481 = new wxStaticText( this, wxID_ANY, wxT("Opacity factor:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText481->Wrap( -1 );
	fgSizer4->Add( m_staticText481, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	OpacityFactorText = new wxTextCtrl( this, wxID_ANY, wxT("1"), wxDefaultPosition, wxSize( 50,-1 ), wxTE_PROCESS_ENTER );
	OpacityFactorText->SetMaxLength( 0 ); 
	fgSizer4->Add( OpacityFactorText, 0, wxALL, 5 );
	
	m_staticText721 = new wxStaticText( this, wxID_ANY, wxT("Retained opacity:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText721->Wrap( -1 );
	fgSizer4->Add( m_staticText721, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer46;
	bSizer46 = new wxBoxSizer( wxHORIZONTAL );
	
	RetainedOpacitySlider = new wxSlider( this, wxID_ANY, 100, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	bSizer46->Add( RetainedOpacitySlider, 0, wxALL, 5 );
	
	RetainedOpacityText = new wxTextCtrl( this, wxID_ANY, wxT("1"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	RetainedOpacityText->SetMaxLength( 0 ); 
	bSizer46->Add( RetainedOpacityText, 0, wxALL, 5 );
	
	
	fgSizer4->Add( bSizer46, 0, 0, 5 );
	
	
	bSizer48->Add( fgSizer4, 0, 0, 5 );
	
	
	BoundaryEnhancementStyle->Add( bSizer48, 1, wxEXPAND, 5 );
	
	
	this->SetSizer( BoundaryEnhancementStyle );
	this->Layout();
	
	// Connect Events
	BoundaryOpacityText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( BoundaryEnhancementVolumeStyleOptions::OnBoundaryOpacityText ), NULL, this );
	OpacityFactorText->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( BoundaryEnhancementVolumeStyleOptions::OnOpacityFactorText ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
}

BoundaryEnhancementVolumeStyleOptions::~BoundaryEnhancementVolumeStyleOptions()
{
	// Disconnect Events
	BoundaryOpacityText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( BoundaryEnhancementVolumeStyleOptions::OnBoundaryOpacityText ), NULL, this );
	OpacityFactorText->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( BoundaryEnhancementVolumeStyleOptions::OnOpacityFactorText ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	RetainedOpacitySlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( BoundaryEnhancementVolumeStyleOptions::OnRetainedOpacityScroll ), NULL, this );
	
}

BlendedVolumeStyleOptions::BlendedVolumeStyleOptions( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	BlendedVolumeStyleSizer = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Blended style") ), wxVERTICAL );
	
	wxBoxSizer* bSizer20111;
	bSizer20111 = new wxBoxSizer( wxHORIZONTAL );
	
	m_button9 = new wxButton( this, wxID_ANY, wxT("Load blend data.."), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer20111->Add( m_button9, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	DensityDataLoadedFileText111 = new wxStaticText( this, wxID_ANY, wxT("Blend data render style:"), wxDefaultPosition, wxDefaultSize, 0 );
	DensityDataLoadedFileText111->Wrap( -1 );
	bSizer20111->Add( DensityDataLoadedFileText111, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxArrayString BlendedVolumeStyleStyleChoiceChoices;
	BlendedVolumeStyleStyleChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, BlendedVolumeStyleStyleChoiceChoices, 0 );
	BlendedVolumeStyleStyleChoice->SetSelection( 0 );
	bSizer20111->Add( BlendedVolumeStyleStyleChoice, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	BlendedVolumeStyleSizer->Add( bSizer20111, 0, 0, 5 );
	
	wxBoxSizer* bSizer201;
	bSizer201 = new wxBoxSizer( wxHORIZONTAL );
	
	n = new wxStaticText( this, wxID_ANY, wxT("Loaded file:"), wxDefaultPosition, wxDefaultSize, 0 );
	n->Wrap( -1 );
	bSizer201->Add( n, 0, wxALL, 5 );
	
	LoadedFileText = new wxStaticText( this, wxID_ANY, wxT("None"), wxDefaultPosition, wxDefaultSize, 0 );
	LoadedFileText->Wrap( -1 );
	bSizer201->Add( LoadedFileText, 0, wxALL, 5 );
	
	
	BlendedVolumeStyleSizer->Add( bSizer201, 0, wxEXPAND, 5 );
	
	wxFlexGridSizer* fgSizer9;
	fgSizer9 = new wxFlexGridSizer( 4, 3, 0, 0 );
	fgSizer9->SetFlexibleDirection( wxBOTH );
	fgSizer9->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText96 = new wxStaticText( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText96->Wrap( -1 );
	fgSizer9->Add( m_staticText96, 0, wxALL, 5 );
	
	m_staticText97 = new wxStaticText( this, wxID_ANY, wxT("Volume"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText97->Wrap( -1 );
	fgSizer9->Add( m_staticText97, 0, wxALL, 5 );
	
	m_staticText98 = new wxStaticText( this, wxID_ANY, wxT("Blend volume"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText98->Wrap( -1 );
	fgSizer9->Add( m_staticText98, 0, wxALL, 5 );
	
	m_staticText93 = new wxStaticText( this, wxID_ANY, wxT("Weight function:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText93->Wrap( -1 );
	fgSizer9->Add( m_staticText93, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString BlendFunction1ChoiceChoices[] = { wxT("CONSTANT"), wxT("TABLE"), wxT("ALPHA1"), wxT("ALPHA2"), wxT("ONE_MINUS_ALPHA1"), wxT("ONE_MINUS_ALPHA2") };
	int BlendFunction1ChoiceNChoices = sizeof( BlendFunction1ChoiceChoices ) / sizeof( wxString );
	BlendFunction1Choice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, BlendFunction1ChoiceNChoices, BlendFunction1ChoiceChoices, 0 );
	BlendFunction1Choice->SetSelection( 0 );
	fgSizer9->Add( BlendFunction1Choice, 0, wxALL, 5 );
	
	wxString BlendFunction2ChoiceChoices[] = { wxT("CONSTANT"), wxT("TABLE"), wxT("ALPHA1"), wxT("ALPHA2"), wxT("ONE_MINUS_ALPHA1"), wxT("ONE_MINUS_ALPHA2") };
	int BlendFunction2ChoiceNChoices = sizeof( BlendFunction2ChoiceChoices ) / sizeof( wxString );
	BlendFunction2Choice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, BlendFunction2ChoiceNChoices, BlendFunction2ChoiceChoices, 0 );
	BlendFunction2Choice->SetSelection( 0 );
	fgSizer9->Add( BlendFunction2Choice, 0, wxALL, 5 );
	
	m_staticText95 = new wxStaticText( this, wxID_ANY, wxT("Weight constant:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText95->Wrap( -1 );
	fgSizer9->Add( m_staticText95, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer46;
	bSizer46 = new wxBoxSizer( wxHORIZONTAL );
	
	BlendConstant1Slider = new wxSlider( this, wxID_ANY, 50, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	bSizer46->Add( BlendConstant1Slider, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	BlendConstant1Text = new wxTextCtrl( this, wxID_ANY, wxT("0.5"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	BlendConstant1Text->SetMaxLength( 0 ); 
	bSizer46->Add( BlendConstant1Text, 0, wxALL, 5 );
	
	
	fgSizer9->Add( bSizer46, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer461;
	bSizer461 = new wxBoxSizer( wxHORIZONTAL );
	
	BlendConstant2Slider = new wxSlider( this, wxID_ANY, 50, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	bSizer461->Add( BlendConstant2Slider, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	BlendConstant2Text = new wxTextCtrl( this, wxID_ANY, wxT("0.5"), wxDefaultPosition, wxSize( 40,-1 ), wxTE_PROCESS_ENTER|wxTE_READONLY );
	BlendConstant2Text->SetMaxLength( 0 ); 
	bSizer461->Add( BlendConstant2Text, 0, wxALL, 5 );
	
	
	fgSizer9->Add( bSizer461, 1, wxEXPAND, 5 );
	
	m_staticText100 = new wxStaticText( this, wxID_ANY, wxT("Weight table:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText100->Wrap( -1 );
	fgSizer9->Add( m_staticText100, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxFlexGridSizer* fgSizer101;
	fgSizer101 = new wxFlexGridSizer( 1, 2, 0, 90 );
	fgSizer101->SetFlexibleDirection( wxBOTH );
	fgSizer101->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	BlendTable1UrlText = new wxStaticText( this, wxID_ANY, wxT("None"), wxDefaultPosition, wxDefaultSize, 0 );
	BlendTable1UrlText->Wrap( -1 );
	fgSizer101->Add( BlendTable1UrlText, 1, wxALL|wxEXPAND|wxALIGN_CENTER_VERTICAL, 5 );
	
	BlendTable1LoadButton = new wxButton( this, wxID_ANY, wxT(".."), wxDefaultPosition, wxSize( 20,20 ), 0 );
	fgSizer101->Add( BlendTable1LoadButton, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	
	fgSizer9->Add( fgSizer101, 1, wxEXPAND, 5 );
	
	wxFlexGridSizer* fgSizer10;
	fgSizer10 = new wxFlexGridSizer( 1, 2, 0, 90 );
	fgSizer10->SetFlexibleDirection( wxBOTH );
	fgSizer10->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	BlendTable2UrlText = new wxStaticText( this, wxID_ANY, wxT("None"), wxDefaultPosition, wxDefaultSize, 0 );
	BlendTable2UrlText->Wrap( -1 );
	fgSizer10->Add( BlendTable2UrlText, 1, wxALL|wxEXPAND|wxALIGN_CENTER_VERTICAL, 5 );
	
	BlendTable2LoadButton = new wxButton( this, wxID_ANY, wxT(".."), wxDefaultPosition, wxSize( 20,20 ), 0 );
	fgSizer10->Add( BlendTable2LoadButton, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	
	fgSizer9->Add( fgSizer10, 1, wxEXPAND, 5 );
	
	
	BlendedVolumeStyleSizer->Add( fgSizer9, 0, wxEXPAND, 5 );
	
	
	this->SetSizer( BlendedVolumeStyleSizer );
	this->Layout();
	
	// Connect Events
	m_button9->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnLoadBlendDataButton ), NULL, this );
	BlendedVolumeStyleStyleChoice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendedVolumeStyleStyleChoice ), NULL, this );
	BlendFunction1Choice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendFunction1Choice ), NULL, this );
	BlendFunction2Choice->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendFunction2Choice ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendTable1LoadButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendTable1LoadButton ), NULL, this );
	BlendTable2LoadButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendTable2LoadButton ), NULL, this );
}

BlendedVolumeStyleOptions::~BlendedVolumeStyleOptions()
{
	// Disconnect Events
	m_button9->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnLoadBlendDataButton ), NULL, this );
	BlendedVolumeStyleStyleChoice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendedVolumeStyleStyleChoice ), NULL, this );
	BlendFunction1Choice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendFunction1Choice ), NULL, this );
	BlendFunction2Choice->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendFunction2Choice ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant1Slider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant1Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendConstant2Slider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( BlendedVolumeStyleOptions::OnBlendConstant2Scroll ), NULL, this );
	BlendTable1LoadButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendTable1LoadButton ), NULL, this );
	BlendTable2LoadButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( BlendedVolumeStyleOptions::OnBlendTable2LoadButton ), NULL, this );
	
}

LoadRawImageDialog::LoadRawImageDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer64;
	bSizer64 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer65;
	bSizer65 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText67 = new wxStaticText( this, wxID_ANY, wxT("Width:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText67->Wrap( -1 );
	bSizer65->Add( m_staticText67, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	WidthSpin = new wxSpinCtrl( this, wxID_ANY, wxT("128"), wxDefaultPosition, wxSize( 50,-1 ), wxSP_ARROW_KEYS, 1, 10000, 128 );
	bSizer65->Add( WidthSpin, 0, wxALL, 5 );
	
	m_staticText68 = new wxStaticText( this, wxID_ANY, wxT("Height:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText68->Wrap( -1 );
	bSizer65->Add( m_staticText68, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	HeightSpin = new wxSpinCtrl( this, wxID_ANY, wxT("128"), wxDefaultPosition, wxSize( 50,-1 ), wxSP_ARROW_KEYS, 1, 10000, 128 );
	bSizer65->Add( HeightSpin, 0, wxALL, 5 );
	
	m_staticText69 = new wxStaticText( this, wxID_ANY, wxT("Depth:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText69->Wrap( -1 );
	bSizer65->Add( m_staticText69, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	DepthSpin = new wxSpinCtrl( this, wxID_ANY, wxT("128"), wxDefaultPosition, wxSize( 50,-1 ), wxSP_ARROW_KEYS, 1, 10000, 128 );
	bSizer65->Add( DepthSpin, 0, wxALL, 5 );
	
	
	bSizer64->Add( bSizer65, 0, 0, 5 );
	
	wxBoxSizer* bSizer66;
	bSizer66 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer67;
	bSizer67 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer66->Add( bSizer67, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer671;
	bSizer671 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer66->Add( bSizer671, 0, 0, 5 );
	
	wxBoxSizer* bSizer71;
	bSizer71 = new wxBoxSizer( wxHORIZONTAL );
	
	
	bSizer66->Add( bSizer71, 0, 0, 5 );
	
	wxFlexGridSizer* fgSizer6;
	fgSizer6 = new wxFlexGridSizer( 4, 2, 0, 0 );
	fgSizer6->SetFlexibleDirection( wxBOTH );
	fgSizer6->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	m_staticText70 = new wxStaticText( this, wxID_ANY, wxT("Pixel type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText70->Wrap( -1 );
	fgSizer6->Add( m_staticText70, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString PixelTypeChoiceChoices[] = { wxT("LUMINANCE"), wxT("LUMINANCE_ALPHA"), wxT("RGB"), wxT("RGBA"), wxT("BGR"), wxT("BGRA") };
	int PixelTypeChoiceNChoices = sizeof( PixelTypeChoiceChoices ) / sizeof( wxString );
	PixelTypeChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxSize( 100,-1 ), PixelTypeChoiceNChoices, PixelTypeChoiceChoices, 0 );
	PixelTypeChoice->SetSelection( 0 );
	fgSizer6->Add( PixelTypeChoice, 0, wxALL, 5 );
	
	m_staticText701 = new wxStaticText( this, wxID_ANY, wxT("Pixel component type:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText701->Wrap( -1 );
	fgSizer6->Add( m_staticText701, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString PixelComponentTypeChoiceChoices[] = { wxT("UNSIGNED"), wxT("SIGNED"), wxT("RATIONAL") };
	int PixelComponentTypeChoiceNChoices = sizeof( PixelComponentTypeChoiceChoices ) / sizeof( wxString );
	PixelComponentTypeChoice = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxSize( 100,-1 ), PixelComponentTypeChoiceNChoices, PixelComponentTypeChoiceChoices, 0 );
	PixelComponentTypeChoice->SetSelection( 0 );
	fgSizer6->Add( PixelComponentTypeChoice, 0, wxALL, 5 );
	
	m_staticText73 = new wxStaticText( this, wxID_ANY, wxT("Bits per pixel:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText73->Wrap( -1 );
	fgSizer6->Add( m_staticText73, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	BitsPerPixelSpin = new wxSpinCtrl( this, wxID_ANY, wxT("8"), wxPoint( -1,-1 ), wxSize( 50,-1 ), wxSP_ARROW_KEYS, 1, 10000, 8 );
	fgSizer6->Add( BitsPerPixelSpin, 0, wxALL, 5 );
	
	m_staticText74 = new wxStaticText( this, wxID_ANY, wxT("Pixel size(in metres):"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText74->Wrap( -1 );
	fgSizer6->Add( m_staticText74, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer72;
	bSizer72 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText75 = new wxStaticText( this, wxID_ANY, wxT("x"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText75->Wrap( -1 );
	bSizer72->Add( m_staticText75, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	PixelSizeXText = new wxTextCtrl( this, wxID_ANY, wxT("0.01"), wxDefaultPosition, wxSize( 40,-1 ), 0 );
	PixelSizeXText->SetMaxLength( 0 ); 
	bSizer72->Add( PixelSizeXText, 0, wxALL, 5 );
	
	m_staticText76 = new wxStaticText( this, wxID_ANY, wxT("y"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText76->Wrap( -1 );
	bSizer72->Add( m_staticText76, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	PixelSizeYText = new wxTextCtrl( this, wxID_ANY, wxT("0.01"), wxDefaultPosition, wxSize( 40,-1 ), 0 );
	PixelSizeYText->SetMaxLength( 0 ); 
	bSizer72->Add( PixelSizeYText, 0, wxALL, 5 );
	
	m_staticText77 = new wxStaticText( this, wxID_ANY, wxT("z"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText77->Wrap( -1 );
	bSizer72->Add( m_staticText77, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	PixelSizeZText = new wxTextCtrl( this, wxID_ANY, wxT("0.01"), wxDefaultPosition, wxSize( 40,-1 ), 0 );
	PixelSizeZText->SetMaxLength( 0 ); 
	bSizer72->Add( PixelSizeZText, 0, wxALL, 5 );
	
	
	fgSizer6->Add( bSizer72, 1, wxEXPAND, 5 );
	
	
	bSizer66->Add( fgSizer6, 0, 0, 5 );
	
	
	bSizer64->Add( bSizer66, 0, 0, 5 );
	
	m_staticline1 = new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
	bSizer64->Add( m_staticline1, 0, wxEXPAND | wxALL, 5 );
	
	wxBoxSizer* bSizer73;
	bSizer73 = new wxBoxSizer( wxHORIZONTAL );
	
	OKButton = new wxButton( this, wxID_ANY, wxT("OK"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer73->Add( OKButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	CancelButton = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer73->Add( CancelButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	
	bSizer64->Add( bSizer73, 1, wxALIGN_CENTER_HORIZONTAL, 5 );
	
	
	this->SetSizer( bSizer64 );
	this->Layout();
	
	// Connect Events
	OKButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LoadRawImageDialog::OnOKButtonPressed ), NULL, this );
	CancelButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LoadRawImageDialog::OnCancelButtonPressed ), NULL, this );
}

LoadRawImageDialog::~LoadRawImageDialog()
{
	// Disconnect Events
	OKButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LoadRawImageDialog::OnOKButtonPressed ), NULL, this );
	CancelButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LoadRawImageDialog::OnCancelButtonPressed ), NULL, this );
	
}
