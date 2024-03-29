#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#include <cairo/cairo.h>
#include "common/time.h"
#include "io/color.h"
#include "io/file_writer.h"

namespace carto_slam
{
  using namespace common;
  namespace io
  {

    // The only cairo image format we use for Cartographer.
    constexpr cairo_format_t kCairoFormat = CAIRO_FORMAT_ARGB32;

    // std::unique_ptr for Cairo surfaces. The surface is destroyed when the
    // std::unique_ptr is reset or destroyed.
    using UniqueCairoSurfacePtr = std::unique_ptr<cairo_surface_t, void (*)(cairo_surface_t *)>;

    // Takes ownership.
    UniqueCairoSurfacePtr MakeUniqueCairoSurfacePtr(cairo_surface_t *surface);

    // std::unique_ptr for Cairo contexts.
    using UniqueCairoPtr = std::unique_ptr<cairo_t, void (*)(cairo_t *)>;

    // Takes ownership.
    UniqueCairoPtr MakeUniqueCairoPtr(cairo_t *surface);

    class Image
    {
    public:
      explicit Image(UniqueCairoSurfacePtr surface);
      Image(int width, int height);

      const Uint8Color GetPixel(int x, int y) const;
      void SetPixel(int x, int y, const Uint8Color &color);
      void WritePng(FileWriter *const file_writer);

      // Rotates the image in place.
      void Rotate90DegreesClockwise();

      // Returns a pointer to a cairo surface that contains the current pixel data.
      // The 'Image' object must therefore outlive the returned surface object. It
      // is undefined behavior to call any of the mutating functions while a pointer
      // to this surface is alive.
      UniqueCairoSurfacePtr GetCairoSurface();

      int width() const { return width_; }
      int height() const { return height_; }

    private:
      int width_;
      int height_;
      std::vector<uint32> pixels_;
    };

  } // namespace io
} // namespace carto_slam
