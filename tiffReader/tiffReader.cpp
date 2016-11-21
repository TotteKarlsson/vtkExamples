//#include "stdafx.h"
#include <string>
//#include "vtkCylinderSource.h"
//#include "vtkPolyDataMapper.h"
//#include "vtkActor.h"


#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkProperty.h"
#include "vtkCamera.h"
#include "vtkTiffReader.h"
#include "vtkSmartPointer.h"
#include "vtkSmartVolumeMapper.h"
#include "vtkVolumeProperty.h"
#include "vtkColorTransferFunction.h"
#include "vtkPiecewiseFunction.h"
#include "vtkImageViewer2.h"
//#include "vtkRayCastImageDisplayHelper.h"

const std::string FILENAME = "p:\\ParaView\\TDTomato_stack.tif"; // a 16-bit multi-page TIFF
																 //const std::string FILENAME = "p:\\ParaView\\Image_0000.tif"; // a 16-bit multi-page TIFF

double F = 1.0;  // for good image; set to 1, ghostly; set to 1000000 opaque
double XY_RESOLUTION = F;
double Z_RESOLUTION = F ;

int main(int argc, char* argv[])
{
	vtkSmartPointer<vtkTIFFReader> reader =	vtkSmartPointer<vtkTIFFReader>::New();
	reader->SetFileName(FILENAME.c_str());
	reader->SetFileDimensionality(3);

	reader->SetDataSpacing(XY_RESOLUTION, XY_RESOLUTION, Z_RESOLUTION);
	reader->SpacingSpecifiedFlagOn(); // to make SetDataSpacing work!!!									  

	vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderer>	 ren1 = vtkSmartPointer<vtkRenderer>::New();

	renWin->AddRenderer(ren1);

	vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(renWin);

	renWin->Render(); // make sure we have an OpenGL context.

	vtkSmartPointer<vtkSmartVolumeMapper> volumeMapper = vtkSmartPointer<vtkSmartVolumeMapper>::New();

	volumeMapper->SetBlendModeToMaximumIntensity();// SetBlendModeToComposite();

	volumeMapper->SetInputConnection(reader->GetOutputPort());
	vtkSmartPointer<vtkVolumeProperty> volumeProperty = vtkSmartPointer<vtkVolumeProperty>::New();
	//volumeProperty->ShadeOff();
	
	//volumeProperty->SetInterpolationType(VTK_LINEAR_INTERPOLATION);
	volumeProperty->SetInterpolationType(VTK_CUBIC_INTERPOLATION);

	vtkSmartPointer<vtkPiecewiseFunction> compositeOpacity = vtkSmartPointer<vtkPiecewiseFunction>::New();
	compositeOpacity->AddPoint(0.0, 0.0);
	compositeOpacity->AddPoint(12000.0, 0.001);
	compositeOpacity->AddPoint(65535.0, 1.0);
	volumeProperty->SetScalarOpacity(compositeOpacity);

	
	vtkSmartPointer<vtkColorTransferFunction> color = vtkSmartPointer<vtkColorTransferFunction>::New();
	
	color->AddRGBPoint(0.0, 0.0, 0.0, 0.0);
	color->AddRGBPoint(65535.0, 1.0, 1.0, 1.0);
	volumeProperty->SetColor(color);


	vtkSmartPointer<vtkVolume> volume = vtkSmartPointer<vtkVolume>::New();
	volume->SetMapper(volumeMapper);
	volume->SetProperty(volumeProperty);
	ren1->AddViewProp(volume);
	ren1->ResetCamera();

	renWin->SetSize(500, 500);
	renWin->SetPosition(0, 0);

	ren1->SetBackground(0.25, 0.25, 0.5);
	ren1->GetActiveCamera()->Elevation(30);

	renWin->Render();

	iren->Start();

	return EXIT_SUCCESS;
}