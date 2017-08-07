#include "main.h"

int main(void)
{
	// Use standard parameters
	quaternion_julia_set_params p;

	// We will march along a finite 3D grid using these step sizes
	const float x_step_size = (p.x_grid_max - p.x_grid_min) / (p.x_res - 1);
	const float y_step_size = (p.y_grid_max - p.y_grid_min) / (p.y_res - 1);
	const float z_step_size = (p.z_grid_max - p.z_grid_min) / (p.z_res - 1);

	// Set up initial Z values
	quaternion Z(p.x_grid_min, p.y_grid_min, p.z_grid_min, p.z_w);
	
	// Set up C values as const since they won't change
	const quaternion C(p.c_x, p.c_y, p.c_z, p.c_w);

	// Holds the float quaternion length data that will be converted to 8-bit char data later on
	vector<float> xyplane(p.x_res*p.y_res, 0);



	// Get maximum slice number width, plus one for an extra space character
	ostringstream max_number_width_oss;
	max_number_width_oss << (p.z_res - 1);
	size_t max_number_width = max_number_width_oss.str().length() + 1;

	// Generate manifest file
	ofstream manifest_file("manifest.xml");

	manifest_file << "<?xml version=\"1.0\"?>" << endl;
	manifest_file << "<grid gridSizeX = \"" << p.x_res << "\" gridSizeY = \"" << p.y_res << "\" gridSizeZ = \"" << p.z_res << "\" voxelSize = \"1.0E-4\" subvoxelBits = \"8\">" << endl;
	manifest_file << " <channels>" << endl;
	manifest_file << "  <channel type = \"DENSITY\" slices = \"density/slice%" << max_number_width << "d.png\"/>" << endl;
	manifest_file << " </channels>" << endl;
	manifest_file << "</grid>" << endl;



	// For each z slice
	for (size_t z = 0; z < p.z_res; z++, Z.z += z_step_size)
	{
		Z.x = p.x_grid_min;

		cout << "Calculating slice " << z + 1 << " of " << p.z_res << endl;

		// Calculate xy plane quaternion length data
		for (size_t x = 0; x < p.x_res; x++, Z.x += x_step_size)
		{
			Z.y = p.y_grid_min;

			for (size_t y = 0; y < p.y_res; y++, Z.y += y_step_size)
			{
				// Make a blank border so that the edges form a closed manifold
				if (x == 0 || x == p.x_res - 1 || y == 0 || y == p.y_res - 1 || z == 0 || z == p.z_res - 1)
					xyplane[x*p.y_res + y] = 2.0f * p.threshold;
				else
					xyplane[x*p.y_res + y] = classic_iter(Z, C, p.max_iterations, p.threshold);
			}

		}

		// Scale and translate the quaternion lengths from [0, infinity] to [0, 1]
		for (size_t i = 0; i < xyplane.size(); i++)
		{
			if (xyplane[i] > 2.0f * p.threshold)
				xyplane[i] = 2.0f * p.threshold;

			xyplane[i] /= 2.0f*p.threshold;
			xyplane[i] = 1.0f - xyplane[i];
		}


		// Output to PNG files
		ostringstream file_name_oss;
		file_name_oss << z;

		size_t number_fill = max_number_width - file_name_oss.str().length();

		string png_file_name = "slice";
		
		for (size_t i = 0; i < number_fill; i++)
			png_file_name += " ";

		png_file_name += file_name_oss.str();
		png_file_name += ".png";

		// Holds the bitmap char data that will be used to generate a PNG file
		vector<unsigned char> png_data;
		png_data.resize(p.x_res*p.y_res);

		// Convert float data to char data
		for (size_t x = 0; x < p.x_res; x++)
			for (size_t y = 0; y < p.y_res; y++)
				png_data[x*p.y_res + y] = static_cast<unsigned char>(255 * xyplane[x*p.y_res + y]);


		// Save the PNG file. Thanks to Lode Vandevenne for the code used to save the PNG file
		// Encode the image. if there's an error, display it
		// we're going to encode with a state rather than a convenient function, because enforcing a color type requires setting options
		lodepng::State state;
		// input color type  (make sure to set this to what your input is, e.g. RGBA+8 below assumes your raw input is 4 bytes per pixel)
		state.info_raw.colortype = LCT_GREY;
		state.info_raw.bitdepth = 8;
		// output color type
		state.info_png.color.colortype = LCT_GREY;
		state.info_png.color.bitdepth = 8;
		state.encoder.auto_convert = 0; // without this, it would ignore the output color type specified above and choose an optimal one instead

		std::vector<unsigned char> buffer;
		unsigned error = lodepng::encode(buffer, &png_data[0], p.x_res, p.y_res, state);

		if (error)
			std::cout << "encoder error " << error << ": " << lodepng_error_text(error) << std::endl;
		else
			lodepng::save_file(buffer, png_file_name.c_str());

	}




	return 0;
}