#include "VolumeModel.h"

using namespace std;
using namespace FS;

#ifdef Q_OS_WIN
extern const char *nrrdBiffKey = "nrrd";
#endif

// Add this node to the H3DNodeDatabase system.
H3D::H3DNodeDatabase VolumeModel::database(
        "VolumeModel",
        &(newInstance<VolumeModel>),
        typeid(VolumeModel),
        &X3DChildNode::database);

namespace VolumeModelInternals
{
H3D::FIELDDB_ELEMENT(VolumeModel, imageField,                    INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, materialSegmentationField,     INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, forbiddenSegmentationField,    INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, partitionField,        		 INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, expertDrillingStepsField,      INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, timeStampsField,            INITIALIZE_ONLY );
H3D::FIELDDB_ELEMENT(VolumeModel, materialSegmentationModelField,INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, forbiddenSegmentationModelField,INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, partitionModelField,   		 INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, nrrdOffset,                    INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, expertFraction,                INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, doMaskPartition,               INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, multiplyImageVoxels,           INPUT_OUTPUT );
H3D::FIELDDB_ELEMENT(VolumeModel, doSaveTimestamps,              INPUT_OUTPUT );
}

/****************************************************************/
VolumeModel::VolumeModel(
        H3D::Inst<VolumeModel::SFTexture3DNode>    _imageField,
        H3D::Inst< MFUnsignedInt8FromNrrdFile >    _materialSegmentationField,
        H3D::Inst< MFUnsignedInt8FromNrrdFile >    _forbiddenSegmentationField,
        H3D::Inst< MFUnsignedInt8FromNrrdFile >    _partitionField,
        H3D::Inst< MFUnsignedInt8FromNrrdFile >    _expertDrillingStepsField,
        H3D::Inst< MFDoubleFromNrrdFile >          _timeStampField,
        H3D::Inst< H3D::SFNode >                   _materialSegmentationModelField,
        H3D::Inst< H3D::SFNode >                   _forbiddenSegmentationModelField,
        H3D::Inst< H3D::SFNode >                   _partitionModelField,
        H3D::Inst< H3D::SFVec3f >                  _nrrdOffset,
        H3D::Inst< H3D::MFFloat >                  _expertFraction,
        H3D::Inst< VolumeModel::SFStringParent >   _doMaskPartion,
        H3D::Inst< H3D::SFFloat >                  _multiplyImageVoxels,
        H3D::Inst< H3D::SFBool >                   _doSaveTimestamps
    ):imageField(_imageField),
      materialSegmentationField(_materialSegmentationField),
      forbiddenSegmentationField(_forbiddenSegmentationField),
      partitionField(_partitionField),
      expertDrillingStepsField(_expertDrillingStepsField),
      timeStampsField(_timeStampField),
      materialSegmentationModelField(_materialSegmentationModelField),
      forbiddenSegmentationModelField(_forbiddenSegmentationModelField),
      partitionModelField(_partitionModelField),
      nrrdOffset(_nrrdOffset),
      expertFraction(_expertFraction),
      doMaskPartition(_doMaskPartion),
      originalImageData(0), multiplyImageVoxels(_multiplyImageVoxels),
      doSaveTimestamps(_doSaveTimestamps)
{
    cout << "Volume Model Constructing"<<endl;
    
    multiplyImageVoxels->setValue(1.0); // Defualt

    for(int i=0;i<256;++i)
        expert_segment_count[i]=0;
       
    type_name = "VolumeModel";
    database.initFields( this );

    // images not loaded in a separate thread
    H3D::X3DTextureNode::load_images_in_separate_thread = false;

    // Init fraction
    std::vector<float> fraction(256);
    for(int s=0;s<256;++s){
        fraction[s] = 0;
    }
    expertFraction->setValue(fraction);

    doSaveTimestamps->setValue(false);


    cout << "Volume Model Constructed"<<endl<<endl;
}

/****************************************************************/
VolumeModel::~VolumeModel()
{
  if(originalImageData)
    delete[] originalImageData;
  if( startTime )
    delete startTime;

  cout << "Volume Model Destructed"<<endl;
}

/****************************************************************/
void VolumeModel::initialize()
{
    cout<<"Volume Model Initializing"<<endl;

    startTime = new H3DUtil::TimeStamp();

    // make a copy of the image data so we can restore this info if needed
    H3D::Image3DTexture *texture = static_cast< H3D::Image3DTexture * >(imageField->getValue());
    imageData = (texture->image)->getValue();

    // TODO: Something wrong here? 2014-08-12
    width  = imageData->width();
    height = imageData->height();
    depth  = imageData->depth();
    
    maxx = width -1;
    maxy = height -1;
    maxz = depth -1;
 
    // a cube is generated around the voxels that are removed so that when the screen is updated
    // only the voxels in this cube are updated
    imageUpdatedOnScreen = true;
    
    size = (unsigned long)width*height*depth;
    spacing = (imageData->pixelSize()).x;

    // save a copy of the image in its original condition
    originalImageData = new H3DUtil::RGBA[size];
    float m = multiplyImageVoxels->getValue();
    for(unsigned long i = 0; i < size; i++)
    {
        int z=i/(height*width);
        int y=(i-z*height*width)/width;
        int x=i-z*height*width-y*width;

        H3DUtil::RGBA v = imageData->getPixel(x,y,z) * m;
        originalImageData[i] = v;
        imageData->setPixel(v,x,y,z);
    }

    // Volume Model is dependent on Material Segmentation Model so insure first that such a child node exists
    if(materialSegmentationModelField->getValue()==NULL)
        throw VolumeModelException ("Expected Volume Model to have a Material Segmentation Model child node");

    // validate segment data
    if(materialSegmentationField->empty()) 	// material segmentation field has not been initialized from a nrrd file
    {
        cout<<"Initializing Material Segmentation Data according to Image file dimensions.";
        cout<<"All voxels given a segment Id of: "<< AIR_SEGMENT_ID;
        materialSegmentationField->setValueFromImageData(imageData,AIR_SEGMENT_ID);
    }
    else				          			// segment field has been initialized from a nrrd file
    {
        if(materialSegmentationField->GetNoOfDimensions()!=3)
            throw VolumeModelException ("Expected the Material Segmentation Data to be 3 dimensional");

        if(imageData->width()!=materialSegmentationField->GetDimensionLengths()[0])
            throw VolumeModelException ("Expected Image Data and Material Segmentation Data to have the same width");

        if(imageData->height()!=materialSegmentationField->GetDimensionLengths()[1])
            throw VolumeModelException ("Expected Image Data and Material Segmentation Data to have the same height");

        if(imageData->depth()!=materialSegmentationField->GetDimensionLengths()[2])
            throw VolumeModelException ("Expected Image Data and Material Segmentation Data to have the same depth");

        if((imageData->pixelSize()).x !=(float)(materialSegmentationField->GetSpacings()[0]))
            throw VolumeModelException ("Expected Image Data and Material Segmentation Data to have the same spacing in the x-axis");

        if((imageData->pixelSize()).y !=(float)(materialSegmentationField->GetSpacings()[1]))
            throw VolumeModelException ("Expected Image Data and Material Segmentation Data to have the same spacing in the y-axis");

        if((imageData->pixelSize()).z !=(float)(materialSegmentationField->GetSpacings()[2]))
            throw VolumeModelException ("Expected Image Data and Material Segmentation Data to have the same spacing in the z-axis");

        double w = imageData->width() * (imageData->pixelSize()).x;
        double h = imageData->height() * (imageData->pixelSize()).y;
        double d = imageData->depth() * (imageData->pixelSize()).z;
        double* space = materialSegmentationField->spaceOrigin;
        nrrdOffset->setValue(Vec3f(float(w/2+space[0]),
                                   float(h/2+space[1]),
                                   float(d/2+space[2])));
        cout << "OFFSET: " << space[0] << ", " <<  space[1] << ", " << space[2] << "\n";
        cout << "w,h,d: " << w<< ", " <<  h << ", " << d << "\n";

    }


    // Volume Model is dependent on Forbidden Segmentation Model so insure first that such a child node exists
    if(forbiddenSegmentationModelField->getValue()==NULL)
        throw VolumeModelException ("Expected Volume Model to have a Forbidden Segmentation Model child node");

    // validate segment data
    if(forbiddenSegmentationField->empty()) 	// forbidden segmentation field has not been initialized from a nrrd file
    {
        cout<<"Initializing Forbidden Segmentation Data according to Image file dimensions.";
        cout<<"All voxels given a segment Id of: "<< 0;
        forbiddenSegmentationField->setValueFromImageData(imageData,0);
    }
    else				          			// segment field has been initialized from a nrrd file
    {

    }


    // Volume Model is dependent on Partition Model so insure first that such a child node exists
    if(partitionModelField->getValue()==NULL)
        throw VolumeModelException ("Expected Volume Model to have a Partition Model child node");

    // validate partition data
    if(partitionField->empty())   // partition field has not been initialized from a nrrd file
    {
        cout<<"Initializing Partition Data according to Image file dimensions. ";
        cout<<"All voxels given a partition Id of: "<<UNDEFINED_PARTITION_ID<<endl;
        partitionField->setValueFromImageData(imageData,UNDEFINED_PARTITION_ID);
    }
    else                        // partition field has been initialized from a nrrd file
    {

    }


    // validate expert drilling data
    if(expertDrillingStepsField->empty())
    {
        cout<<"Initializing Expert Drilling Steps Data according to Image file dimensions"<<endl;
        expertDrillingStepsField->setValueFromImageData(imageData,0);
    }
    else
    {
        if(expertDrillingStepsField->GetNoOfDimensions()!=3)
            throw VolumeModelException ("Expected the Expert Drilling Steps Data to be 3 dimensional");

        if(imageData->width()!=expertDrillingStepsField->GetDimensionLengths()[0])
            throw VolumeModelException ("Expected Image Data and Expert Drilling Steps Data to have the same width");

        if(imageData->height()!=expertDrillingStepsField->GetDimensionLengths()[1])
            throw VolumeModelException ("Expected Image Data and Expert Drilling Steps Data to have the same height");

        if(imageData->depth()!=expertDrillingStepsField->GetDimensionLengths()[2])
            throw VolumeModelException ("Expected Image Data and Expert Drilling Steps Data to have the same depth");

        if((imageData->pixelSize()).x !=(float)(expertDrillingStepsField->GetSpacings()[0]))
            throw VolumeModelException ("Expected Image Data and Expert Drilling Steps Data to have the same spacing in the x-axis");

        if((imageData->pixelSize()).y !=(float)(expertDrillingStepsField->GetSpacings()[1]))
            throw VolumeModelException ("Expected Image Data and Expert Drilling Steps Data to have the same spacing in the y-axis");

        if((imageData->pixelSize()).z !=(float)(expertDrillingStepsField->GetSpacings()[2]))
            throw VolumeModelException ("Expected Image Data and Expert Drilling Steps Data to have the same spacing in the z-axis");

        for(int i=0;i<255;++i) expert_segment_max[i]=0;
        for(unsigned int i=0;i<imageData->width()*imageData->height()*imageData->depth();++i){
            expert_segment_max[expertDrillingStepsField->getValue()[i]]++;
            //std::cout << int(expertDrillingStepsField->getValue()[i]) << ": "
            //          << expert_segment_max[expertDrillingStepsField->getValue()[i]] << "\n";
        }
    }

    // validate time stamp data
    if(timeStampsField->empty())
    {
        cout<<"Initializing Time Stamp Data according to Image file dimensions"<<endl;
        timeStamps.reset(new MFDoubleFromNrrdFile());
        timeStamps->setValueFromImageData(imageData,10000);
    }
    else
    {
        timeStamps.reset(new MFDoubleFromNrrdFile(timeStampsField.get()));

        /*if(timeStamps->GetNoOfDimensions()!=3)
            throw VolumeModelException ("Expected the Time Stamp Data to be 3 dimensional");

        if(imageData->width()!=timeStamps->GetDimensionLengths()[0])
            throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same width");
        ForbiddenSegmentationModel* forbiddenSegmentationModel = static_cast< ForbiddenSegmentationModel * >(forbiddenSegmentationModelField->getValue());
        const std::vector<unsigned char> segmentIds = forbiddenSegmentationModel->getSegmentIds();

        if(imageData->height()!=timeStamps->GetDimensionLengths()[1])
            throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same height");

        if(imageData->depth()!=timeStamps->GetDimensionLengths()[2])
            throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same depth");

        if((imageData->pixelSize()).x !=(float)(timeStamps->GetSpacings()[0]))
            throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same spacing in the x-axis");

        if((imageData->pixelSize()).y !=(float)(timeStamps->GetSpacings()[1]))
            throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same spacing in the y-axis");

        if((imageData->pixelSize()).z !=(float)(timeStamps->GetSpacings()[2]))
            throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same spacing in the z-axis");*/
    }

    // create a material remaining field
    cout<<"Initializing Material Remaining Data according to Segment data"<<endl;
    materialSegmentationField->GetMaskedVersion(materialRemainingField);
    
    global = H3D::Vec3f(0,0,0);
    relative = H3D::Vec3f(0,0,0);
    indexZeroPos = H3D::Vec3f(0-(maxx/2.0f)*spacing,
                              0-(maxy/2.0f)*spacing,
                              0-(maxz/2.0f)*spacing); // Volume centered around 0,0,0

    // Create a multimap vector from the timestamp-file. This will make it much faster to do volume-
    // playback since it will not be necessary to traverse the whole volume for each new image-update.
    for(unsigned int i =0;i<width;i++)
        for(unsigned int j =0;j<height;j++)
            for(unsigned int k =0;k<depth;k++)
            {
                Vec3i vec(i,j,k);
                double timestamp = getTimeStamp(vec);
                if(timestamp!=-1.0)
                    timestamp_map.insert(pair<double,Vec3i>(timestamp, vec));
            }


    // Cache parition box boundaries
    for(int i=0;i<5;++i){ // max 5 partitions...
        getPartionBox(i);
    }
     



     X3DChildNode::initialize();

     cout<<"Volume Model Initialized"<<endl<<endl;
}



/****************************************************************/
void VolumeModel::traverseSG(H3D::TraverseInfo&)
{
    // update the region on screen where we have bored
    H3DUtil::TimeStamp now = H3DUtil::TimeStamp::now();
    if((double)now > (double)lastUpdateTime + 1/60.0)
    {
        lastUpdateTime = now;
        if(!imageUpdatedOnScreen)
        {
            imageField->getValue()->image->beginEditing();
            imageField->getValue()->image->setEditedArea(
                        imageRegionToUpdateOnScreen_minX,
                        imageRegionToUpdateOnScreen_minY,
                        imageRegionToUpdateOnScreen_minZ,
                        imageRegionToUpdateOnScreen_maxX,
                        imageRegionToUpdateOnScreen_maxY,
                        imageRegionToUpdateOnScreen_maxZ);

            imageField->getValue()->image->endEditing();

            imageUpdatedOnScreen = true;
        }
    }

    // Calculate fraction drilled
    std::vector<float> fraction(256);
    for(int s=0;s<256;++s){
        fraction[s] = expert_segment_max[s] == 0 ? 1 :
                      float(expert_segment_count[s])/float(expert_segment_max[s]);
    }
    expertFraction->setValue(fraction);
    /*
    std::cout << "seg max: " << expert_segment_max[0] << " " <<
                                expert_segment_max[1] << " " <<
                                expert_segment_max[2] << " " <<
                                expert_segment_max[3] << " " <<
                                expert_segment_max[4] << " \n";
    std::cout << "seg count: " << expert_segment_count[0] << " " <<
                                expert_segment_count[1] << " " <<
                                expert_segment_count[2] << " " <<
                                expert_segment_count[3] << " " <<
                                expert_segment_count[4] << " \n";
                                */

    // Save to file if this is requested
    if(doSaveTimestamps->getValue()){
        H3DUtil::TimeStamp currentTime;
        // Only allow saving every 10s, this to prevent multiple calls
        if(currentTime-lastSaveTime>10.0){
            lastSaveTime = currentTime;
            saveTimeStamps();
        }
        doSaveTimestamps->setValue(false);
    }

}

/****************************************************************/
void VolumeModel::setTimeStampFile(const string& nrrdFilePath )
{
    cout << "Setting Timestamp Data from file: " << nrrdFilePath <<endl;

    if(!timeStamps.get()||!imageData)
        return;

    timeStamps->setValueFromString(nrrdFilePath);

    if(timeStamps->GetNoOfDimensions()!=3)
        throw VolumeModelException ("Expected the Time Stamp Data to be 3 dimensional");

    if(imageData->width()!=timeStamps->GetDimensionLengths()[0])
        throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same width");

    if(imageData->height()!=timeStamps->GetDimensionLengths()[1])
        throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same height");

    if(imageData->depth()!=timeStamps->GetDimensionLengths()[2])
        throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same depth");

    if((imageData->pixelSize()).x !=(float)(timeStamps->GetSpacings()[0]))
        throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same spacing in the x-axis");

    if((imageData->pixelSize()).y !=(float)(timeStamps->GetSpacings()[1]))
        throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same spacing in the y-axis");

    if((imageData->pixelSize()).z !=(float)(timeStamps->GetSpacings()[2]))
        throw VolumeModelException ("Expected Image Data and Time Stamp Data to have the same spacing in the z-axis");

    timestamp_map.clear();
    // Create a multimap vector from the timestamp-file. This will make it much faster to do volume-
    // playback since it will not be necessary to traverse whole volume for each new image-update.
    for(unsigned int i =0;i<width;i++)
        for(unsigned int j =0;j<height;j++)
            for(unsigned int k =0;k<depth;k++)
            {
                Vec3i vec(i,j,k);
                double timestamp = getTimeStamp(vec);
                if(timestamp!=-1)
                    timestamp_map.insert(pair<double,Vec3i>(timestamp, vec));
            }
}

/****************************************************************/
double VolumeModel::getElapsedTime()
{
    H3DUtil::TimeStamp currentTime;
    return currentTime - *startTime;
}

/****************************************************************/
void VolumeModel::resetStartTime()
{
    //Starttime is set to current time when drilling has started, see ADrillableNode
    if( startTime )
        delete startTime;

    startTime = new H3DUtil::TimeStamp();
}

/****************************************************************/
unsigned int VolumeModel::getHeight()
{
    return height;
}

/****************************************************************/
unsigned int VolumeModel::getWidth()
{
    return width;
}

/****************************************************************/
unsigned int VolumeModel::getDepth()
{
    return depth;
}

/****************************************************************/
float VolumeModel::getSpacing()
{
    return spacing;
}

/****************************************************************/
unsigned int VolumeModel::getIndex(Vec3i p)
{
    return ((unsigned int)p.z * height * width + (unsigned int)p.y * width + (unsigned int)p.x);
}


/****************************************************************/
VolumeModel::minmax VolumeModel::getPartionBox(int partitionId)
{
    if(!partitionBox.count(partitionId)){
        minmax m;
        m.min.x = width;
        m.min.y = height;
        m.min.z = depth;
        m.max.x = 0;
        m.max.y = 0;
        m.max.z = 0;

        for(unsigned int i =0;i<width;i++){
            for(unsigned int j =0;j<height;j++){
                for(unsigned int k =0;k<depth;k++){
                    Vec3i vec(i,j,k);
                    if(getPartitionId(vec)==partitionId){
                        if(vec.x < m.min.x) m.min.x = vec.x;
                        if(vec.y < m.min.y) m.min.y = vec.y;
                        if(vec.z < m.min.z) m.min.z = vec.z;

                        if(vec.x > m.max.x) m.max.x = vec.x;
                        if(vec.y > m.max.y) m.max.y = vec.y;
                        if(vec.z > m.max.z) m.max.z = vec.z;
                    }
                }
            }
        }
        partitionBox[partitionId] = m;
    }
    return partitionBox[partitionId];
}

/****************************************************************/
void VolumeModel::setScreenUpdateArea(Vec3i minp, Vec3i maxp)
{
	using std::max;
	using std::min;
	typedef unsigned int ui;
	int padding = 1;
	
	// update the area edited so that minp ans maxp are included
	if(imageUpdatedOnScreen==true)
	{
		imageRegionToUpdateOnScreen_maxX = min(ui(maxp.x + padding), maxx);
		imageRegionToUpdateOnScreen_maxY = min(ui(maxp.y + padding), maxy);
		imageRegionToUpdateOnScreen_maxZ = min(ui(maxp.z + padding), maxz);
		imageRegionToUpdateOnScreen_minX = max(minp.x - padding, 0);
		imageRegionToUpdateOnScreen_minY = max(minp.y - padding, 0);
		imageRegionToUpdateOnScreen_minZ = max(minp.z -padding, 0);
		
		imageUpdatedOnScreen = false;
	}
	else
	{
		if(maxp.x > imageRegionToUpdateOnScreen_maxX)
			imageRegionToUpdateOnScreen_maxX = min(ui(maxp.x + padding), maxx);
		
		if(minp.x < imageRegionToUpdateOnScreen_minX)
			imageRegionToUpdateOnScreen_minX = max(minp.x - padding, 0);

		if(maxp.y > imageRegionToUpdateOnScreen_maxY)
			imageRegionToUpdateOnScreen_maxY = min(ui(maxp.y + padding), maxy);
		
		if(minp.y < imageRegionToUpdateOnScreen_minY)
			imageRegionToUpdateOnScreen_minY = max(minp.y - padding, 0);

		if(maxp.z > imageRegionToUpdateOnScreen_maxZ)
			imageRegionToUpdateOnScreen_maxZ = min(ui(maxp.z + padding), maxz);
		
		if(minp.z < imageRegionToUpdateOnScreen_minZ)
			imageRegionToUpdateOnScreen_minZ = max(minp.z - padding, 0);
	}
}

/****************************************************************/
void VolumeModel::setImageValue(Vec3i p, float r, float g, float b)
{
	setScreenUpdateArea(p,p);
	imageData->setPixel(H3D::RGBA(r,g,b,1),p.x,p.y,p.z);
}


/****************************************************************/
void VolumeModel::setImageValue(Vec3i p, float r, float g, float b, bool setScreenArea)
{
	if (setScreenArea)
		setScreenUpdateArea(p,p);
	
    imageData->setPixel(H3D::RGBA(r,g,b,1),p.x,p.y,p.z);
}


/****************************************************************/
H3D::H3DFloat VolumeModel::getImageValue(Vec3i p)
{
    return imageData->getPixel(p.x,p.y,p.z).r;
}

H3D::H3DFloat VolumeModel::getImageValue(unsigned int i)
{
    int z=i/(height*width);
    int y=(i-z*height*width)/width;
    int x=i-z*height*width-y*width;
    return imageData->getPixel(x,y,z).r;
}

H3D::H3DFloat VolumeModel::getOriginalImageValue(Vec3i p)
{
    return originalImageData[getIndex(p)].r;
}

/****************************************************************/
void VolumeModel::setMaterialSegmentationId(Vec3i p, unsigned char value)
{
    materialSegmentationField->setValue(getIndex(p),value);
}

/****************************************************************/
unsigned char VolumeModel::getMaterialSegmentationId(Vec3i p)
{
    return materialSegmentationField->getValue()[getIndex(p)];
}

/****************************************************************/
unsigned char VolumeModel::getMaterialSegmentationId(unsigned int index)
{
    return materialSegmentationField->getValue()[index];
}

/****************************************************************/
unsigned char VolumeModel::getForbiddenSegmentationId(Vec3i p)
{
    return forbiddenSegmentationField->getValue()[getIndex(p)];
}

/****************************************************************/
void VolumeModel::incrementNoOfVoxelsBoredByUserForForbiddenSegment(Vec3i p)
{
    ForbiddenSegmentationModel* forbiddenSegmentationModel = static_cast< ForbiddenSegmentationModel * >(forbiddenSegmentationModelField->getValue());
    forbiddenSegmentationModel->incrementNoOfVoxelsBoredByUser(getForbiddenSegmentationId(p));
}

/****************************************************************/
void VolumeModel::decrementNoOfVoxelsBoredByUserForForbiddenSegment(Vec3i p)
{
    ForbiddenSegmentationModel* forbiddenSegmentationModel = static_cast< ForbiddenSegmentationModel * >(forbiddenSegmentationModelField->getValue());
    forbiddenSegmentationModel->decrementNoOfVoxelsBoredByUser(getForbiddenSegmentationId(p));
}


/****************************************************************/
void VolumeModel::incrementNoOfVoxelsBoredByUserForSegment(Vec3i p)
{
    MaterialSegmentationModel* materialSegmentationModel = static_cast< MaterialSegmentationModel * >(materialSegmentationModelField->getValue());
    materialSegmentationModel->incrementNoOfVoxelsBoredByUser(getMaterialSegmentationId(p));
}

/****************************************************************/
void VolumeModel::decrementNoOfVoxelsBoredByUserForSegment(Vec3i p)
{
    MaterialSegmentationModel* materialSegmentationModel = static_cast< MaterialSegmentationModel * >(materialSegmentationModelField->getValue());
    materialSegmentationModel->decrementNoOfVoxelsBoredByUser(getMaterialSegmentationId(p));
}


/****************************************************************/
void VolumeModel::incrementNoOfVoxelsBoredByUserForExpertSegment(Vec3i p)
{
    expert_segment_count[getExpertDrillingStepId(p)]++;
}

/****************************************************************/
void VolumeModel::setPartitionId(Vec3i p, unsigned char value)
{
    partitionField->setValue(getIndex(p),value);
}

/****************************************************************/
unsigned char VolumeModel::getPartitionId(Vec3i p)
{
    return partitionField->getValue()[getIndex(p)];
}

/****************************************************************/
unsigned char VolumeModel::getPartitionId(unsigned int index)
{
    return partitionField->getValue()[index];
}

/****************************************************************/
bool VolumeModel::isExpertDrillingStep(Vec3i p, unsigned char expertDrillingStepId)
{
    return expertDrillingStepsField->getValue()[getIndex(p)]==expertDrillingStepId?true:false;
}

/****************************************************************/
unsigned char VolumeModel::getExpertDrillingStepId(Vec3i p)
{
    return expertDrillingStepsField->getValue()[getIndex(p)];
}

/****************************************************************/
void VolumeModel::setExpertDrillingStepId(Vec3i p, unsigned char drillingStepId)
{
    return expertDrillingStepsField->setValue(getIndex(p), drillingStepId);
}

/****************************************************************/
bool VolumeModel::isDrilledByUser(Vec3i p)
{
    if(materialRemainingField.getValue()[getIndex(p)]==0.0 && materialSegmentationField->getValue()[getIndex(p)]!=AIR_SEGMENT_ID)
        return true;
    return false;
}

/****************************************************************/
void VolumeModel::printUniqueDrilledBy()
{
    vector<unsigned char> list;

    for(unsigned int i =0;i<getWidth();i++)
        for(unsigned int j =0;j<getHeight();j++)
            for(unsigned int k =0;k<getDepth();k++)
            {
                Vec3i vec(i,j,k);
                vector<unsigned char>::iterator iter;
                unsigned char value = expertDrillingStepsField->getValue()[getIndex(vec)];
                bool found = false;
                for(iter=list.begin(); iter<list.end(); iter++)
                    if(*iter == value)
                    {

                        found = true;
                        break;
                    }
                if(!found)
                {
                    list.push_back(value);
                    cout << hex <<(unsigned int)value << endl;
                }
            }
}

/****************************************************************/
void VolumeModel::setMaterialRemaining(Vec3i p, float value)
{	
    unsigned int index = getIndex(p);
	materialRemainingField.setValue(index, value);
}

/****************************************************************/
float VolumeModel::getMaterialRemaining(Vec3i p)
{
    return materialRemainingField.getValue()[getIndex(p)];
}

/****************************************************************/
float VolumeModel::getMaterialRemaining(unsigned int index)
{
    return materialRemainingField.getValue()[index];
}

/****************************************************************/
void VolumeModel::setTimeStamp(Vec3i p, double value)
{
    timeStamps->setValue(getIndex(p),value);
    timestamp_map.insert(pair<double,Vec3i>(value, p));
}

/****************************************************************/
double VolumeModel::getTimeStamp(Vec3i p)
{
    return timeStamps->getValue()[getIndex(p)];
}

/****************************************************************/
double VolumeModel::getTimeStamp(unsigned int index)
{
    return timeStamps->getValue()[index];
}

/****************************************************************/
string VolumeModel::saveTimeStamps(string filename)
{
    cout << "VolumeModel: Saving Timestamps" << endl;

    unsigned int noOfDimensions = 3;

    vector<unsigned int> dimensionLengths;
    dimensionLengths.push_back(width);
    dimensionLengths.push_back(height);
    dimensionLengths.push_back(depth);

    vector<double> spacings;
    spacings.push_back(spacing);
    spacings.push_back(spacing);
    spacings.push_back(spacing);

    // construct string with current date and time
    time_t rawTime = time(0);
    struct tm* timeInfo;
    char strBuffer [80];
    timeInfo = localtime ( &rawTime );
    strftime (strBuffer,80,"%y%m%d%H%M%S",timeInfo);
    //	string timeString(strBuffer);


	try {
    timeStamps->saveDataToNrrdFile(filename,noOfDimensions,dimensionLengths,spacings);
    cout << "Data saved to file: " << filename << endl;
  }
	catch(...) {
    cout << "Error saving data to file: " << filename << endl;
	}
    
    return filename;
}

/****************************************************************/


/****************************************************************/
unsigned int VolumeModel::noOfVoxelsRemoved(unsigned int segmentId){

    //Number of voxels drilled in each segment
    int noOfRemovedVoxels = 0;


    multimap<double, Vec3i>::iterator it;
    for ( it=timestamp_map.begin() ; it !=timestamp_map.end(); it++ )
    {
        if (segmentId == getMaterialSegmentationId((*it).second))
            noOfRemovedVoxels++;
    }
    return noOfRemovedVoxels;
    return noOfRemovedVoxels;
}

/****************************************************************/


/****************************************************************/
H3D::Vec3f VolumeModel::getGlobalCoordinate()
{
    return global;
}

/****************************************************************/
H3D::Vec3f VolumeModel::getGlobalCoordinate(int i)
{
    return getLocalCoordinate(i)+global;
}

/****************************************************************/
H3D::Vec3f VolumeModel::getGlobalCoordinate(Vec3i p)
{
    return getLocalCoordinate(p)+global;
}

/****************************************************************/
H3D::Vec3f VolumeModel::getLocalCoordinate()
{
    return H3D::Vec3f(0,0,0);
}

/****************************************************************/
H3D::Vec3f VolumeModel::getLocalCoordinate(int i)
{
    Vec3i p;
    p.z=i/(height*width);
    p.y=(i-p.z*height*width)/width;
    p.x=i-p.z*height*width-p.y*width;
    return(getLocalCoordinate(p));
}

/****************************************************************/
H3D::Vec3f VolumeModel::getLocalCoordinate(Vec3i p)
{
    return H3D::Vec3f
            (p.x*spacing,p.y*spacing,p.z*spacing)+indexZeroPos;
}

/****************************************************************/
H3D::Vec3f VolumeModel::getRelativeCoordinate()
{
    return getGlobalCoordinate()-relative;
}

/****************************************************************/
H3D::Vec3f VolumeModel::getRelativeCoordinate(int i)
{
    return getGlobalCoordinate(i)-relative;
}

/****************************************************************/
H3D::Vec3f VolumeModel::getRelativeCoordinate(Vec3i p)
{
    return getGlobalCoordinate(p)-relative;
}

// The coordinates of the center of our object, in GLOBAL coordinates
void VolumeModel::setGlobalCoordinate(H3D::Vec3f c)
{
    global = c;
}

/****************************************************************/
// The coordinates of relative (reference) point, in GLOBAL coordinates
void VolumeModel::setRelativeCoordinate(H3D::Vec3f c)
{
    relative = c;
}

/****************************************************************/
Vec3i VolumeModel::getIndexLocal(H3D::Vec3f c)
{
    Vec3i i;
    H3D::Vec3f voxel=(c-indexZeroPos)/spacing;
    i.x = (int)voxel.x;
    i.y = (int)voxel.y;
    i.z = (int)voxel.z;
    if(i.x<0 || i.y<0 || i.z<0 || (unsigned int)i.x>maxx || (unsigned int)i.y>maxy || (unsigned int)i.z>maxz){
        i.x=-1;
        i.y=-1;
        i.z=-1;
    }
    return i;
}

/****************************************************************/
Vec3i VolumeModel::getIndexLocalWall(H3D::Vec3f c)
{
    Vec3i i;
    H3D::Vec3f voxel=(c-indexZeroPos)/spacing;
    i.x = (int)voxel.x;
    i.y = (int)voxel.y;
    i.z = (int)voxel.z;
    return wall(i);
}

/****************************************************************/
Vec3i VolumeModel::getIndexGlobal(H3D::Vec3f c)
{
    return getIndexLocal(c-global);
}

/****************************************************************/
Vec3i VolumeModel::getIndexGlobalWall(H3D::Vec3f c)
{
    return getIndexLocalWall(c-global);
}

/****************************************************************/
Vec3i VolumeModel::getIndexRelative(H3D::Vec3f c)
{
    return getIndexLocal(c-relative);
}

/****************************************************************/
Vec3i VolumeModel::getIndexRelativeWall(H3D::Vec3f c)
{
    return getIndexLocalWall(c-relative);
}

/****************************************************************/
// For safe in-boundary calculations, returns 0 if coordinate < 0, max if
// coordinate above max
Vec3i VolumeModel::wall(Vec3i i)
{
    if(i.x<0) i.x=0;
    if(i.y<0) i.y=0;
    if(i.z<0) i.z=0;
    if((unsigned int)i.x>maxx) i.x=maxx;
    if((unsigned int)i.y>maxy) i.y=maxy;
    if((unsigned int)i.z>maxz) i.z=maxz;
    return i;
}

/****************************************************************/
// Returns a list of 6 cartesian neighbours of given scalar id. If no neighbour
// exists (out of border), id will be set to -1.
void VolumeModel::getNeighbours(Vec3i *vector, Vec3i id)
{
    vector[0]=Vec3i(id.x-1,id.y,id.z);
    if(vector[0].x<0)
        vector[0] = Vec3i(-1,-1,-1);

    vector[1]=Vec3i(id.x+1,id.y,id.z);
    if((unsigned int)vector[1].x>maxx)
        vector[1] = Vec3i(-1,-1,-1);

    vector[2]=Vec3i(id.x,id.y-1,id.z);
    if(vector[2].y<0)
        vector[2] = Vec3i(-1,-1,-1);

    vector[3]=Vec3i(id.x,id.y+1,id.z);
    if((unsigned int)vector[3].y>maxy)
        vector[3] = Vec3i(-1,-1,-1);

    vector[4]=Vec3i(id.x,id.y,id.z-1);
    if(vector[4].z<0)

        vector[5]=Vec3i(id.x,id.y,id.z+1);
    if((unsigned int)vector[5].z>maxz)
        vector[5] = Vec3i(-1,-1,-1);
}

/****************************************************************/
bool VolumeModel::isInsideVolume( H3D::Vec3f c )
{
    H3D::Vec3f max_v( spacing * (maxx+1) / 2,
                      spacing * (maxy+1) / 2,
                      spacing * (maxz+1) / 2 );

    return (c.x >= -max_v.x && c.x <= max_v.x &&
            c.y >= -max_v.y && c.y <= max_v.y &&
            c.z >= -max_v.z && c.z <= max_v.z );
}

/****************************************************************/
void VolumeModel::removePartitionByName(string partitionName, float newAmount)
{
    PartitionModel* partitionModel = static_cast< PartitionModel * >(partitionModelField->getValue());
    unsigned char idOfPartitionToRemove = partitionModel->getId(partitionName);

    double elapsedTime = getElapsedTime();

    minmax m = getPartionBox(idOfPartitionToRemove);

    for(int i =m.min.x;i<=m.max.x;i++)
        for(int j =m.min.y;j<=m.max.y;j++)
            for(int k =m.min.z;k<m.max.z;k++)
            {
                Vec3i vec(i,j,k);
                if(getPartitionId(vec)==idOfPartitionToRemove)
                {
                    if(getImageValue(vec) >= newAmount){
                        setImageValue(vec,newAmount,newAmount,newAmount);
                    }
                    setMaterialRemaining(vec,0);
                    if(getTimeStamp(vec)>9000) // 10000s is the default undrilled timestamp value
                    {
                        //The voxel has previously not been removed already
                        setTimeStamp(vec,elapsedTime);
                    }
                    //setAsCurrentExpertDrillingStep(vec);
                    //setThatUserHasBoredThisDrillingStep(vec,elapsedTime);
                }
            }
}

/****************************************************************/
float VolumeModel::getMaxImageVal()
{
    float maxVal = imageData ? (float)(pow(2, imageData->bitsPerPixel()) -1) : 0.0f;
	return maxVal;
}

/****************************************************************/
float VolumeModel::getMaxSegVal()
{
	MaterialSegmentationModel* materialSegmentationModel = static_cast< MaterialSegmentationModel * >(materialSegmentationModelField->getValue());
    float maxSegVal = materialSegmentationModel ? float(materialSegmentationModel->getMaxSegmentValue()) : 0.0f;
	return maxSegVal;
}
