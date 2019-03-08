///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Nov  6 2013)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __MEDX3DDEMO_H__
#define __MEDX3DDEMO_H__

#include <wx/artprov.h>
#include <wx/xrc/xmlres.h>
#include <wx/string.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/frame.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/statbox.h>
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/clrpicker.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/notebook.h>
#include <wx/dialog.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/statline.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class MainFrame
///////////////////////////////////////////////////////////////////////////////
class MainFrame : public wxFrame 
{
	private:
	
	protected:
		wxMenuBar* MainMenuBar;
		wxMenu* FileMenu;
		wxMenu* WindowsMenu;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClose( wxCloseEvent& event ) { event.Skip(); }
		virtual void OnIdle( wxIdleEvent& event ) { event.Skip(); }
		virtual void OnLoadVolumeData( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnLoadRawData( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnQuit( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnColorVolumeStyleMenuItem( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnConsoleWindowsMenu( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		wxMenuItem* VolumeStyleMenuItem;
		wxMenuItem* ConsoleWindowsMenu;
		
		MainFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("SenseGraphics MedX3D Demo"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 604,550 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		
		~MainFrame();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class MainDialog
///////////////////////////////////////////////////////////////////////////////
class MainDialog : public wxDialog 
{
	private:
	
	protected:
		wxNotebook* m_notebook2;
		wxPanel* m_panel3;
		wxStaticBoxSizer* DensityDataStaticBoxSizer;
		wxButton* LoadButton;
		wxButton* LoadRawButton;
		wxStaticText* m_staticText3;
		wxGridSizer* DensityDataInfoSizer;
		wxStaticText* m_staticText10;
		wxStaticText* DensityDataDimensionsText;
		wxStaticText* m_staticText12;
		wxStaticText* DensityDataVoxelSizeText;
		wxStaticText* m_staticText22;
		wxStaticText* DensityDataComponentsText;
		wxStaticText* m_staticText101;
		wxStaticText* DensityDataBitsPerVoxelText;
		wxStaticText* m_staticText121;
		wxStaticText* DensityDataTypeText;
		wxStaticText* m_staticText51;
		wxStaticText* DensityDataMinValueText;
		wxStaticText* m_staticText53;
		wxStaticText* DensityDataMaxValueText;
		wxPanel* RenderOptionsPanel;
		wxBoxSizer* VolumeDataSizer;
		wxStaticText* m_staticText58;
		wxChoice* m_choice8;
		wxStaticText* m_staticText60;
		wxChoice* RendererChoice;
		wxStaticText* m_staticText61;
		wxColourPickerCtrl* m_colourPicker19;
		wxStaticText* m_staticText59;
		wxTextCtrl* RayStepText;
		wxCheckBox* m_checkBox13;
		wxCheckBox* m_checkBox15;
		wxStaticText* m_staticText57;
		wxChoice* m_choice7;
		wxCheckBox* m_checkBox14;
		wxCheckBox* m_checkBox16;
		wxStaticText* m_staticText591;
		wxTextCtrl* NrSlicesText;
		wxPanel* RenderStylesPanel;
		wxBoxSizer* RenderStylesSizer;
		wxStaticText* m_staticText78;
		wxChoice* m_choice18;
		wxButton* m_button101;
		wxButton* m_button14;
		wxStaticBoxSizer* VolumeDataNodeSizer;
		wxStaticText* m_staticText222121;
		wxChoice* VolumeDataRenderStyleChoice;
		wxStaticBoxSizer* SegmentedDataNodeSizer;
		wxButton* m_button9;
		wxStaticText* m_staticText31;
		wxStaticText* m_staticText3111;
		wxChoice* SegmentChoice;
		wxChoice* SegmentRenderStyleChoice;
		wxCheckBox* SegmentEnabledCheck;
		wxStaticBoxSizer* IsoSurfacesNodeSizer;
		wxStaticText* m_staticText312;
		wxTextCtrl* IsoValuesText;
		wxStaticText* m_staticText31111;
		wxChoice* IsoSurfaceChoice;
		wxChoice* IsoSurfaceRenderStyleChoice;
		wxStaticText* m_staticText3121;
		wxTextCtrl* ContourStepSizeText;
		wxStaticText* m_staticText3122;
		wxTextCtrl* SurfaceToleranceText;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnCloseDialog( wxCloseEvent& event ) { event.Skip(); }
		virtual void OnIdle( wxIdleEvent& event ) { event.Skip(); }
		virtual void OnLoadButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnLoadRawButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSaveAsNrrdButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnDataFilterChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnRendererChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBackgroundColorChanged( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnRayStep( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnUseEmptySpaceSkipping( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnShowNonEmptySpace( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnEmptySpaceResolution( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnStopRaysAtGeom( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnUseStochasticJittering( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnNrSlices( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnVolumeNodeChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnShowStyleEditor( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSaveAsX3DButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnRenderStyleChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnLoadSegmentDataButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSegmentChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSegmentStyleChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSegmentEnabledCheck( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnIsoValueChange( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnIsoSurfaceChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnIsoSurfaceStyleChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnContourStepSizeChange( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSurfaceToleranceChange( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		wxButton* SaveAsNrrdButton;
		wxStaticText* DensityDataLoadedFileText;
		wxStaticText* LoadedSegmentDataText;
		wxStaticText* DensityDataLoadedFileText111;
		wxStaticText* DensityDataLoadedFileText1111;
		
		MainDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxEmptyString, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 447,510 ), long style = wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER ); 
		~MainDialog();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class StyleDialog
///////////////////////////////////////////////////////////////////////////////
class StyleDialog : public wxDialog 
{
	private:
	
	protected:
		wxStaticText* m_staticText2222;
		wxChoice* StyleNameChoice;
		wxButton* m_button7;
		wxButton* m_button8;
		wxButton* SaveStyleButton;
		wxStaticText* m_staticText222121;
		wxChoice* StyleTypeChoice;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnRenderStyleChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnNewStyleButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnDeleteStyleButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSaveStyleButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnStyleTypeChoice( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		wxBoxSizer* StyleDialogMainSizer;
		
		StyleDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Style editor.."), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 510,590 ), long style = wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER ); 
		~StyleDialog();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class ShadedVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class ShadedVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticBoxSizer* ShadedVolumeStyleOptions1;
		wxCheckBox* ShadedOnMaterialCheck;
		wxStaticText* DiffuseColorText;
		wxColourPickerCtrl* DiffuseColorPicker;
		wxStaticText* EmissiveColorText;
		wxColourPickerCtrl* EmissiveColorPicker;
		wxStaticText* SpecularColorText;
		wxColourPickerCtrl* SpecularColorPicker;
		wxStaticText* ShadedTransparencyLabel;
		wxSlider* ShadedTransparencySlider;
		wxTextCtrl* ShadedTransparencyText;
		wxStaticText* ShadedAmbientIntensityLabel;
		wxSlider* ShadedAmbientIntensitySlider;
		wxTextCtrl* ShadedAmbientIntensityText;
		wxCheckBox* ShadedLightingBox;
		wxCheckBox* ShadedShadowsBox;
		wxStaticText* m_staticText611;
		wxChoice* m_choice51;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnShadedUseMaterialCheck( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnShadedDiffuseColorChanged( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnShadedEmissiveColorChanged( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnShadedSpecularColorChanged( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnShadedTransparencyScroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnShadedAmbientIntensityScroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnShadedLightingBox( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnShadedShadowsBox( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		wxFlexGridSizer* ShadedMaterialWidgetsSizer;
		
		ShadedVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 470,196 ), long style = wxTAB_TRAVERSAL ); 
		~ShadedVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class OpacityMapVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class OpacityMapVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticBoxSizer* OpacityMapStyleOptions;
		wxStaticText* m_staticText61;
		wxChoice* TypeChoice;
		wxStaticText* m_staticText77;
		wxChoice* TransferFunctionChoice;
		wxBoxSizer* FileOptionsSizer;
		wxButton* m_button20;
		wxStaticText* m_staticText78;
		wxStaticText* FilenameText;
		wxFlexGridSizer* WindowOptionsSizer;
		wxStaticText* m_staticText80;
		wxSlider* WindowCenterSlider;
		wxStaticText* WindowCenterText;
		wxStaticText* m_staticText801;
		wxSlider* WindowWidthSlider;
		wxStaticText* WindowWidthText;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnTypeChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnTransferFunctionChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnLoadImageButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnWindowCenterScroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnWindowWidthScroll( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		OpacityMapVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 418,194 ), long style = wxTAB_TRAVERSAL ); 
		~OpacityMapVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class ToneMappedVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class ToneMappedVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText35;
		wxColourPickerCtrl* CoolColorPicker;
		wxStaticText* m_staticText36;
		wxColourPickerCtrl* WarmColorPicker;
		wxStaticText* m_staticText86;
		wxSlider* ToneMappedCoolColorTransparencySlider;
		wxTextCtrl* ToneMappedCoolColorTransparencyText;
		wxStaticText* m_staticText861;
		wxSlider* ToneMappedWarmColorTransparencySlider;
		wxTextCtrl* ToneMappedWarmColorTransparencyText;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnCoolColorChanged( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnWarmColorChanged( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnCoolColorTransparencyScroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnWarmColorTransparencyScroll( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		ToneMappedVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 397,107 ), long style = wxTAB_TRAVERSAL ); 
		~ToneMappedVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class CartoonVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class CartoonVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText35;
		wxColourPickerCtrl* ParallelColorPicker;
		wxStaticText* m_staticText36;
		wxColourPickerCtrl* OrthogonalColorPicker;
		wxStaticText* m_staticText86;
		wxSlider* CartoonParallelColorTransparencySlider;
		wxTextCtrl* CartoonParallelColorTransparencyText;
		wxStaticText* m_staticText861;
		wxSlider* CartoonOrthogonalColorTransparencySlider;
		wxTextCtrl* CartoonOrthogonalColorTransparencyText;
		wxStaticText* m_staticText721;
		wxSlider* NrStepsSlider;
		wxTextCtrl* NrStepsText;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnParallelColorChange( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnOrthogonalColorChange( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnParallelColorTransparencyScroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnOrthogonalColorTransparencyScroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnStepsScroll( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		CartoonVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 493,139 ), long style = wxTAB_TRAVERSAL ); 
		~CartoonVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class MIPVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class MIPVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText74;
		wxChoice* ProjectionStyleTypeChoice;
		wxStaticText* m_staticText48;
		wxSlider* IntensityThresholdSlider;
		wxTextCtrl* IntensityThresholdText;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnProjectionStyleTypeChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnIntensityThresholdSlider( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		MIPVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 498,75 ), long style = wxTAB_TRAVERSAL ); 
		~MIPVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class ComposedVolumeStyleOptionsPart
///////////////////////////////////////////////////////////////////////////////
class ComposedVolumeStyleOptionsPart : public wxPanel 
{
	private:
	
	protected:
		wxFlexGridSizer* FlexSizer;
		wxBoxSizer* OptionsSizer;
		wxPanel* m_panel7;
		wxButton* m_button13;
		wxButton* m_button14;
		wxButton* m_button15;
		wxButton* m_button16;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnAddStyleButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnRemoveStyleButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMoveStyleUpButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnMoveStyleDownButton( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		ComposedVolumeStyleOptionsPart( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 498,75 ), long style = wxTAB_TRAVERSAL ); 
		~ComposedVolumeStyleOptionsPart();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class ComposedVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class ComposedVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxBoxSizer* StylePartsSizer;
	
	public:
		
		ComposedVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 498,75 ), long style = wxTAB_TRAVERSAL ); 
		~ComposedVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class SilhouetteEnhancementVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class SilhouetteEnhancementVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText48;
		wxTextCtrl* BoundaryOpacityText;
		wxStaticText* m_staticText481;
		wxTextCtrl* SharpnessText;
		wxStaticText* m_staticText721;
		wxSlider* RetainedOpacitySlider;
		wxTextCtrl* RetainedOpacityText;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnBoundaryOpacityChanged( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnSharpnessChanged( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnRetainedOpacityScroll( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		SilhouetteEnhancementVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 490,142 ), long style = wxTAB_TRAVERSAL ); 
		~SilhouetteEnhancementVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class EdgeEnhancementVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class EdgeEnhancementVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText35;
		wxColourPickerCtrl* EdgeColorPicker;
		wxStaticText* m_staticText46;
		wxSlider* GradientThresholdSlider;
		wxTextCtrl* GradientThresholdText;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnEdgeColorChanged( wxColourPickerEvent& event ) { event.Skip(); }
		virtual void OnGradientThresholdScroll( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		EdgeEnhancementVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 493,111 ), long style = wxTAB_TRAVERSAL ); 
		~EdgeEnhancementVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class BoundaryEnhancementVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class BoundaryEnhancementVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText48;
		wxTextCtrl* BoundaryOpacityText;
		wxStaticText* m_staticText481;
		wxTextCtrl* OpacityFactorText;
		wxStaticText* m_staticText721;
		wxSlider* RetainedOpacitySlider;
		wxTextCtrl* RetainedOpacityText;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnBoundaryOpacityText( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnOpacityFactorText( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnRetainedOpacityScroll( wxScrollEvent& event ) { event.Skip(); }
		
	
	public:
		
		BoundaryEnhancementVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 496,134 ), long style = wxTAB_TRAVERSAL ); 
		~BoundaryEnhancementVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class BlendedVolumeStyleOptions
///////////////////////////////////////////////////////////////////////////////
class BlendedVolumeStyleOptions : public wxPanel 
{
	private:
	
	protected:
		wxStaticBoxSizer* BlendedVolumeStyleSizer;
		wxButton* m_button9;
		wxChoice* BlendedVolumeStyleStyleChoice;
		wxStaticText* n;
		wxStaticText* m_staticText96;
		wxStaticText* m_staticText97;
		wxStaticText* m_staticText98;
		wxStaticText* m_staticText93;
		wxChoice* BlendFunction1Choice;
		wxChoice* BlendFunction2Choice;
		wxStaticText* m_staticText95;
		wxSlider* BlendConstant1Slider;
		wxTextCtrl* BlendConstant1Text;
		wxSlider* BlendConstant2Slider;
		wxTextCtrl* BlendConstant2Text;
		wxStaticText* m_staticText100;
		wxStaticText* BlendTable1UrlText;
		wxButton* BlendTable1LoadButton;
		wxStaticText* BlendTable2UrlText;
		wxButton* BlendTable2LoadButton;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnLoadBlendDataButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBlendedVolumeStyleStyleChoice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBlendFunction1Choice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBlendFunction2Choice( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBlendConstant1Scroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnBlendConstant2Scroll( wxScrollEvent& event ) { event.Skip(); }
		virtual void OnBlendTable1LoadButton( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnBlendTable2LoadButton( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		wxStaticText* DensityDataLoadedFileText111;
		wxStaticText* LoadedFileText;
		
		BlendedVolumeStyleOptions( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 496,220 ), long style = wxTAB_TRAVERSAL ); 
		~BlendedVolumeStyleOptions();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class LoadRawImageDialog
///////////////////////////////////////////////////////////////////////////////
class LoadRawImageDialog : public wxDialog 
{
	private:
	
	protected:
		wxStaticText* m_staticText67;
		wxSpinCtrl* WidthSpin;
		wxStaticText* m_staticText68;
		wxSpinCtrl* HeightSpin;
		wxStaticText* m_staticText69;
		wxSpinCtrl* DepthSpin;
		wxStaticText* m_staticText70;
		wxChoice* PixelTypeChoice;
		wxStaticText* m_staticText701;
		wxChoice* PixelComponentTypeChoice;
		wxStaticText* m_staticText73;
		wxSpinCtrl* BitsPerPixelSpin;
		wxStaticText* m_staticText74;
		wxStaticText* m_staticText75;
		wxTextCtrl* PixelSizeXText;
		wxStaticText* m_staticText76;
		wxTextCtrl* PixelSizeYText;
		wxStaticText* m_staticText77;
		wxTextCtrl* PixelSizeZText;
		wxStaticLine* m_staticline1;
		wxButton* OKButton;
		wxButton* CancelButton;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnOKButtonPressed( wxCommandEvent& event ) { event.Skip(); }
		virtual void OnCancelButtonPressed( wxCommandEvent& event ) { event.Skip(); }
		
	
	public:
		
		LoadRawImageDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Open raw file"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 401,244 ), long style = wxDEFAULT_DIALOG_STYLE ); 
		~LoadRawImageDialog();
	
};

#endif //__MEDX3DDEMO_H__
