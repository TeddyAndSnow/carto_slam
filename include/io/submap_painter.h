#pragma once

#include <Eigen/Geometry>
#include <cairo/cairo.h>
#include "io/image.h"
#include "estimator/id.h"
#include "estimator/value_conversion_tables.h"
#include "common/rigid_transform.h"

namespace carto_slam
{
  namespace io
  {

    struct PaintSubmapSlicesResult
    {
      PaintSubmapSlicesResult(UniqueCairoSurfacePtr surface,
                              Eigen::Array2f origin)
          : surface(std::move(surface)), origin(origin) {}
      UniqueCairoSurfacePtr surface;

      // Top left pixel of 'surface' in map frame.
      Eigen::Array2f origin;
    };

    struct SubmapSlice
    {
      SubmapSlice()
          : surface(MakeUniqueCairoSurfacePtr(nullptr)) {}

      // Texture data.
      int width;
      int height;
      int version;
      double resolution;
      common::Rigid3d slice_pose;
      UniqueCairoSurfacePtr surface;
      // Pixel data used by 'surface'. Must outlive 'surface'.
      std::vector<uint32_t> cairo_data;

      // Metadata.
      common::Rigid3d pose;
      int metadata_version = -1;
    };

    struct SubmapTextureData {
      int width;
      int height;
      double resolution;
      common::Rigid3d slice_pose;
      std::string cells;
    };

    struct SubmapTexture
    {
      struct Pixels
      {
        std::vector<char> intensity;
        std::vector<char> alpha;
      };
      Pixels pixels;
      int width;
      int height;
      double resolution;
      common::Rigid3d slice_pose;
    };

    struct SubmapTextures
    {
      int version;
      std::vector<SubmapTexture> textures;
    };

    PaintSubmapSlicesResult PaintSubmapSlices(
        const std::map<estimator::SubmapId, SubmapSlice> &submaps,
        double resolution);

    // void FillSubmapSlice(
    //     const ::common::Rigid3d& global_submap_pose,
    //     const ::cartographer::mapping::proto::Submap& proto,
    //     SubmapSlice* const submap_slice,
    //     mapping::ValueConversionTables* conversion_tables);

    // void DeserializeAndFillSubmapSlices(
    //     ProtoStreamDeserializer* deserializer,
    //     std::map<::cartographer::mapping::SubmapId, SubmapSlice>* submap_slices,
    //     mapping::ValueConversionTables* conversion_tables);

    // Unpacks cell data as provided by the backend into 'intensity' and 'alpha'.
    SubmapTexture::Pixels UnpackTextureData(const std::string &compressed_cells,
                                            int width, int height);

    // Draw a texture into a cairo surface. 'cairo_data' will store the pixel data
    // for the surface and must therefore outlive the use of the surface.
    UniqueCairoSurfacePtr DrawTexture(const std::vector<char> &intensity,
                                      const std::vector<char> &alpha, int width,
                                      int height,
                                      std::vector<uint32_t> *cairo_data);

  } // namespace io
} // namespace carto_slam
