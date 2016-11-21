#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkCamera.h"
#include "vtkInteractorStyleSwitch.h"
#include "vtkImageReader2.h"
#include "vtkActor.h"

#include "vtkColorTransferFunction.h"
#include "vtkPiecewiseFunction.h"
#include "vtkVolumeProperty.h"
#include "vtkVolumeRayCastCompositeFunction.h"
#include "vtkVolumeRayCastMapper.h"
#include "vtkVolume.h"
#include "vtkTiffReader.h"
#include "vtkFixedPointVolumeRayCastMapper.h"
#include <vtkGPUVolumeRayCastMapper.h>

#define mode_MIP                        0
#define mode_XRay                        1
#define mode_Full                        2

static int currentRenderMode = mode_Full;

// Macro for rounding x (for x >= 0)
inline int vtkRoundFuncMacro(double x)
{
   //return vtkFastNumericConversion::Round(x);
   return x;
}

// Necessary for correct MIP and XRay scaling (0-1)
static float globalMaximumScalarValue = 0;
////////////////////////////////////////////////////////////////
// Definition and implementation of our own ray cast composite function ----------------------------------
class MyVolumeRayCastCompositeFunction : public
vtkVolumeRayCastCompositeFunction
{

public:
    static MyVolumeRayCastCompositeFunction *New()
    { return new MyVolumeRayCastCompositeFunction; }

protected:

    ////////////////////////////////////////////////////////////////
    // Do not change this code
    //
    // This is called from RenderAnImage (in vtkDepthPARCMapper.cxx)
    // It uses the integer data type flag that is passed in
    // to determine what type of ray needs to be cast (which is handled
    // by a template function.  It also uses the shading and
    // interpolation types to determine which template function
    // to call.
    void MyVolumeRayCastCompositeFunction::CastRay( vtkVolumeRayCastDynamicInfo *dynamicInfo, vtkVolumeRayCastStaticInfo *staticInfo )
    {
        void *data_ptr;
        data_ptr = staticInfo->ScalarDataPointer;

        switch ( staticInfo->ScalarDataType ) 
		{
			case VTK_UNSIGNED_CHAR:
				vtkCastRay_OwnWork((unsigned char *)data_ptr, dynamicInfo, staticInfo);
			break;
			case VTK_UNSIGNED_SHORT:
				vtkCastRay_OwnWork((unsigned short *)data_ptr, dynamicInfo, staticInfo);
			break;
			default:
				vtkWarningMacro(<< "Unsigned char and unsigned short are the only supported datatypes for rendering");
			break;
        }
    }

    // This is the template function that actually casts a ray and computes
    // the composite value.  This version uses nearest neighbor interpolation, and performs shading.
    template <class T>
    void vtkCastRay_OwnWork(T *data_ptr, vtkVolumeRayCastDynamicInfo *dynamicInfo, vtkVolumeRayCastStaticInfo *staticInfo )
    {
        unsigned char   *gmptr = NULL;
        float           accum_red_intensity, accum_green_intensity, accum_blue_intensity, accum_opacity;
        float           opacity;
        int             xinc, yinc, zinc;
        int             voxel[3];
        float           ray_position[3];
        T               *dptr;
        float           *ScalarToOpacityTF;
        float           *ScalarToColorTF;
        float           *red_d_shade, *green_d_shade, *blue_d_shade;
        float           *red_s_shade, *green_s_shade, *blue_s_shade;
        unsigned short  *encoded_normals;
        int             currentNormalIndex;
        float           red_shaded_value, green_shaded_value, blue_shaded_value;
        int             offset;
        int             steps_this_ray;
        int             scalar_value;
        float           r, g, b;
        int             num_steps;
        float           *ray_start, *ray_increment;
        int                shading = staticInfo->Shading;


        // For each ray some information are fetched which are needed for color and shading
        // computation.

        // Get ray parameters: Number of sample points, ray start position, and step size on the ray
        num_steps = dynamicInfo->NumberOfStepsToTake;
        ray_start = dynamicInfo->TransformedStart;
        ray_increment = dynamicInfo->TransformedIncrement;

        // Get diffuse shading table pointers. This table is preprocessed by VTK and stores
        // for different normal directions its diffuse portion of the phong illumination term.
        red_d_shade   = staticInfo->RedDiffuseShadingTable;
        green_d_shade = staticInfo->GreenDiffuseShadingTable;
        blue_d_shade  = staticInfo->BlueDiffuseShadingTable;

        // Get specular shading table pointers. This table is preprocessed by VTK and stores
        // for specular normal directions its diffuse portion of the phong illumination term.
        red_s_shade = staticInfo->RedSpecularShadingTable;
        green_s_shade = staticInfo->GreenSpecularShadingTable;
        blue_s_shade = staticInfo->BlueSpecularShadingTable;

        // Get a pointer to the encoded normals of this volume. This table is preprocessed by VTK and stores
        // for each voxel its normal.
        encoded_normals = staticInfo->EncodedNormals;

        // Get the scalar opacity transfer function which maps scalar input values to opacities
        ScalarToOpacityTF = staticInfo->Volume->GetCorrectedScalarOpacityArray();

        // Get the color transfer function which maps scalar input values to RGB values
        ScalarToColorTF =  staticInfo->Volume->GetRGBArray();

        // store increments (distance between sample points in x, y, and z direction) in local variables
        xinc = staticInfo->DataIncrement[0];
        yinc = staticInfo->DataIncrement[1];
        zinc = staticInfo->DataIncrement[2];
        //cerr<<xinc<<", "<<yinc<<", "<<zinc<<", "<<endl; //constant for a data set

        // Initialize the ray position (in voxel space)
        ray_position[0] = ray_start[0];
        ray_position[1] = ray_start[1];
        ray_position[2] = ray_start[2];


        // Nearest neighbor interpolation - get nearest voxel (in voxel space) - you have to change this for tri-linear interpolation
        voxel[0] = vtkRoundFuncMacro( ray_position[0] );
        voxel[1] = vtkRoundFuncMacro( ray_position[1] );
        voxel[2] = vtkRoundFuncMacro( ray_position[2] );

        // So far we haven't accumulated anything
        accum_red_intensity   = 0.0;
        accum_green_intensity = 0.0;
        accum_blue_intensity  = 0.0;
        // accumulated opacity
        accum_opacity         = 0.0;

        // added for MIP mode rendering
        float maxOpacity      = 0.0;
        float maxValue        = 0.0;

        currentNormalIndex    = 0;

        // For each step along the ray do something (accumulate, find maximum opacity, ...)
        // "accum_opacity < 1.0" means early-ray-termination
        for ( steps_this_ray = 0; steps_this_ray < num_steps && accum_opacity < 1.0; steps_this_ray++ ) {

            // get offset position of current voxel in linear list
            offset = voxel[2] * zinc + voxel[1] * yinc + voxel[0];
            // get pointer on current voxel in linear list
            dptr = data_ptr + offset;

            // get scalar value at current location
            scalar_value       = (int) *(dptr);

            if (scalar_value > globalMaximumScalarValue)
            {
                globalMaximumScalarValue = scalar_value;
            }

            // get opacity value for this scalar_value out of the ScalarOpacityTransferFunction
            opacity = ScalarToOpacityTF[scalar_value];

            // If we have a valid opacity value, then compute the shading
            if ( opacity ) {
                switch (currentRenderMode)
                {
                case mode_MIP:
                    // MIP - only maximal value or first local maximum above a threshold is visualized
                    // check if current voxel opacity is above max opacity
                    //if (opacity > maxOpacity)
                    if (scalar_value > maxValue)
                    {
                        accum_red_intensity   = opacity * ScalarToColorTF[(scalar_value) * 3    ];
                        accum_green_intensity = opacity * ScalarToColorTF[(scalar_value) * 3 + 1];
                        accum_blue_intensity  = opacity * ScalarToColorTF[(scalar_value) * 3 + 2];

                        //maxOpacity    = opacity;
                        maxValue    = scalar_value;
                        accum_opacity = 1.0-opacity;

                    }

                    break;
                case mode_XRay:
                    // X-ray projection mode: Accumulation of the scalar values (or opacity values)
                    accum_red_intensity = accum_green_intensity = accum_blue_intensity = accum_red_intensity + scalar_value;

                    break;
                case mode_Full:
                    // get r, g, and b value for current scalar value out of the ColorTransferFunction
                    r = ScalarToColorTF[(scalar_value) * 3    ];
                    g = ScalarToColorTF[(scalar_value) * 3 + 1];
                    b = ScalarToColorTF[(scalar_value) * 3 + 2];

                    red_shaded_value   = opacity * r;
                    green_shaded_value = opacity * g;
                    blue_shaded_value  = opacity * b;


                    // Accumulate color
                    // (the color of the current value is weighted by the remaining opacity of the voxels
                    // in front of the current one) accum_red_intensity   = red_shaded_value   * (1.0-accum_opacity) + accum_red_intensity;
                    accum_green_intensity = green_shaded_value * (1.0-accum_opacity) + accum_green_intensity;
                    accum_blue_intensity  = blue_shaded_value  * (1.0-accum_opacity) + accum_blue_intensity;

                    // Accumulate opacity
                    accum_opacity  = opacity*(1.0-accum_opacity) + accum_opacity;

                    break;
                default:
                    accum_red_intensity = 0.0;
                    accum_green_intensity = 0.0;
                    accum_blue_intensity = 0.0;
                    accum_opacity = 1.0;

                    break;
                }

            }

            // Increment our position and compute new voxel location
            ray_position[0] += ray_increment[0];
            ray_position[1] += ray_increment[1];
            ray_position[2] += ray_increment[2];
            voxel[0] = vtkRoundFuncMacro( ray_position[0] );
            voxel[1] = vtkRoundFuncMacro( ray_position[1] );
            voxel[2] = vtkRoundFuncMacro( ray_position[2] );
        }// endFor each step along the ray ---------------------------------------------------------------------------- -----------


        // Now we stored in accum_{red,green,blue}_intensity the accumulated RGB values along a ray
        // added for XRay mode rendering
        if( currentRenderMode == mode_XRay)
        {
            // divide accumulated scalar values by the number of steps times maximum scalar value to get the average
            accum_red_intensity = accum_green_intensity = accum_blue_intensity = accum_red_intensity / float(num_steps*globalMaximumScalarValue); accum_opacity = 1.0-accum_red_intensity;

        }

        // Cap the accumulated intensities at 1.0
        if ( accum_red_intensity > 1.0 )    { accum_red_intensity = 1.0; }
        if ( accum_green_intensity > 1.0 ){ accum_green_intensity = 1.0;}
        if ( accum_blue_intensity > 1.0 )    { accum_blue_intensity = 1.0;}

        // Set the return pixel value.
        dynamicInfo->Color[0] = accum_red_intensity;
        dynamicInfo->Color[1] = accum_green_intensity;
        dynamicInfo->Color[2] = accum_blue_intensity;
        dynamicInfo->Color[3] = accum_opacity;
        dynamicInfo->NumberOfStepsTaken = steps_this_ray;


    }

};
// ---------------------------------------------------------------------------- -


////////////////////////////////////////////////////////////////
//
int main (int argc, char **argv)
{
    //check for  arguments (e.g. a filename)
    const char* fname = "p:\\ParaView\\TDTomato_stack.tif";

    // Create the renderer, the render window, and the interactor. The renderer
    // draws into the render window, the interactor enables mouse- and
    // keyboard-based interaction with the data within the render window.
    vtkRenderer *renderer = vtkRenderer::New();
    //renderer->SetBackground( 0., 0., 0. );
    renderer->SetBackground( 1.0, 1.0, 1.0 );
    vtkCamera *camera = renderer->GetActiveCamera();
    camera->ParallelProjectionOff();
    camera->SetViewUp (0, 0, -1);
    camera->SetPosition (-1, 2, -0.5);
    vtkRenderWindow *renderWindow = vtkRenderWindow::New();
    renderWindow->SetSize( 640, 480 );     // Set the size of the render window (in pixel).
    renderWindow->AddRenderer(renderer);
    vtkRenderWindowInteractor *interactionRenderer = vtkRenderWindowInteractor::New();
    interactionRenderer->SetRenderWindow(renderWindow);

	reinterpret_cast<vtkInteractorStyleSwitch*>(interactionRenderer->GetInteractorStyle())->SetCurrentStyleToTrackballCamera();

    vtkTIFFReader *volumeDataset = vtkTIFFReader::New();
    volumeDataset->SetFileName(fname);
    volumeDataset->SetFileDimensionality(3);
    volumeDataset->SetDataExtent(0, 255, 0, 255, 0, 255);
    volumeDataset->SetDataSpacing(1., 1., 1.);
	volumeDataset->SpacingSpecifiedFlagOn(); // to make SetDataSpacing work!!!									  
    volumeDataset->SetDataOrigin(0.0, 0.0, 0.0);
    volumeDataset->SetDataScalarTypeToUnsignedShort();
    volumeDataset->SetDataByteOrderToLittleEndian();
    volumeDataset->UpdateWholeExtent();


////////////////////////////////////////////////////////////////////////////
///////////
    // Feel free to change this code
    // adjustments to the following code may be required and should be explored

    // Create transfer function - mapping scalar values [0-...] to COLOR [0-1, 0-1, 0-1]
    vtkColorTransferFunction *colorTransferFunction = vtkColorTransferFunction::New();
    colorTransferFunction->AddRGBPoint(   0.0, 0.0,0.0,0.0);
    colorTransferFunction->AddRGBPoint( 500.0, 0.9,0.5,0.3);
    colorTransferFunction->AddRGBPoint(1100.0, 0.8,0.8,0.6);
    colorTransferFunction->AddRGBPoint(1200.0, 0.6,0.6,0.6);

    vtkPiecewiseFunction *opacityTransferFunction = vtkPiecewiseFunction::New();
    opacityTransferFunction->AddPoint(    0, 0.0);
    //opacityTransferFunction->AddPoint(  980, 0.1);
    //opacityTransferFunction->AddPoint(  1055, 0.2);
    opacityTransferFunction->AddPoint(  1200, 0.05);
    opacityTransferFunction->AddPoint( 1300, 1.0);

    // The property describes how the data will look
    vtkVolumeProperty *volumeProperty = vtkVolumeProperty::New();
    volumeProperty->SetColor(colorTransferFunction);
    volumeProperty->SetScalarOpacity(opacityTransferFunction);
    //volumeProperty->ShadeOn(); // request 3d shading (german: beleuchtungsberechnung anfordern) //has to be implemented in vtkCastRay_OwnWork

    //vtkVolumeRayCastCompositeFunction *rayCompositeFunction = vtkVolumeRayCastCompositeFunction::New();
    MyVolumeRayCastCompositeFunction *rayCompositeFunction = MyVolumeRayCastCompositeFunction::New();
    
	// The mapper knows how to render the data	
    //vtkVolumeRayCastMapper *rayCastMapper = vtkVolumeRayCastMapper::New();
	//vtkFixedPointVolumeRayCastMapper *rayCastMapper = vtkFixedPointVolumeRayCastMapper::New();
	vtkFixedPointVolumeRayCastMapper *rayCastMapper = vtkFixedPointVolumeRayCastMapper::New();
	//vtkGPUVolumeRayCastMapper *rayCastMapper = vtkGPUVolumeRayCastMapper::New();

    rayCastMapper->SetInputData( volumeDataset->GetOutput());
    //rayCastMapper->SetVSetVolumeRayCastFunction(rayCompositeFunction);
	rayCastMapper->SetBlendModeToMaximumIntensity();
    //Feel free to change parameters for
    //rayCastMapper->AutoAdjustSampleDistancesOff();        // interactive framerates by lower quality (on/off)
    rayCastMapper->SetSampleDistance(1.0);
	// adjust samples per ray rate if AutoAdjustSampleDistances is OFF
    //rayCastMapper->SetImageSampleDistance(1.0);            // adjust rays per pixel rate (e.g. 0.5 => 4 rays for each pixel, 2.0 => 1 ray for 4 pixel) if AutoAdjustSampleDistances is OFF
    //rayCastMapper->SetMaximumImageSampleDistance(6.0);    // max sample distance if AutoAdjustSampleDistances is ON
    //rayCastMapper->SetMinimumImageSampleDistance(1.0);    // min sample distance if AutoAdjustSampleDistances is ON

    // The volume holds the mapper and the property and
    // can be used to position/orient the volume
    vtkVolume *volData= vtkVolume::New();
    volData->SetMapper(rayCastMapper);
    volData->SetProperty(volumeProperty);

    // Actors are added to the renderer.
    renderer->AddVolume(volData);


    // adjust camera for good initial view
    renderer->ResetCamera();
    renderer->GetActiveCamera()->Dolly(1);
    renderer->ResetCameraClippingRange();

    // Start the Interactor
    interactionRenderer->Initialize();
    interactionRenderer->Start();

    // It is important to delete all objects created previously to prevent
    // memory leaks. In this case, since the program is on its way to
    // exiting, it is not so important. But in applications it is
    // essential.
    renderer->Delete();
    renderWindow->Delete();
    interactionRenderer->Delete();

    return 0;
}
