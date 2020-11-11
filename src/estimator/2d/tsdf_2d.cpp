
#include "estimator/2d/tsdf_2d.h"
#include "io/fast_zip.h"
#include <memory>

namespace carto_slam
{
  namespace estimator
  {

    TSDF2D::TSDF2D(const MapLimits &limits, float truncation_distance,
                   float max_weight, ValueConversionTables *conversion_tables)
        : Grid2D(limits, -truncation_distance, truncation_distance,
                 conversion_tables),
          conversion_tables_(conversion_tables),
          value_converter_(std::make_unique<TSDValueConverter>(
              truncation_distance, max_weight, conversion_tables_)),
          weight_cells_(
              limits.cell_limits().num_x_cells * limits.cell_limits().num_y_cells,
              value_converter_->getUnknownWeightValue()) {}

    bool TSDF2D::CellIsUpdated(const Eigen::Array2i &cell_index) const
    {
      const int flat_index = ToFlatIndex(cell_index);
      uint16 tsdf_cell = correspondence_cost_cells()[flat_index];
      return tsdf_cell >= value_converter_->getUpdateMarker();
    }

    void TSDF2D::SetCell(const Eigen::Array2i &cell_index, float tsd,
                         float weight)
    {
      const int flat_index = ToFlatIndex(cell_index);
      uint16 *tsdf_cell = &(*mutable_correspondence_cost_cells())[flat_index];
      if (*tsdf_cell >= value_converter_->getUpdateMarker())
      {
        return;
      }
      mutable_update_indices()->push_back(flat_index);
      mutable_known_cells_box()->extend(cell_index.matrix());
      *tsdf_cell =
          value_converter_->TSDToValue(tsd) + value_converter_->getUpdateMarker();
      uint16 *weight_cell = &weight_cells_[flat_index];
      *weight_cell = value_converter_->WeightToValue(weight);
    }

    GridType TSDF2D::GetGridType() const { return GridType::TSDF; }

    float TSDF2D::GetTSD(const Eigen::Array2i &cell_index) const
    {
      if (limits().Contains(cell_index))
      {
        return value_converter_->ValueToTSD(
            correspondence_cost_cells()[ToFlatIndex(cell_index)]);
      }
      return value_converter_->getMinTSD();
    }

    float TSDF2D::GetWeight(const Eigen::Array2i &cell_index) const
    {
      if (limits().Contains(cell_index))
      {
        return value_converter_->ValueToWeight(
            weight_cells_[ToFlatIndex(cell_index)]);
      }
      return value_converter_->getMinWeight();
    }

    std::pair<float, float> TSDF2D::GetTSDAndWeight(
        const Eigen::Array2i &cell_index) const
    {
      if (limits().Contains(cell_index))
      {
        int flat_index = ToFlatIndex(cell_index);
        return std::make_pair(
            value_converter_->ValueToTSD(correspondence_cost_cells()[flat_index]),
            value_converter_->ValueToWeight(weight_cells_[flat_index]));
      }
      return std::make_pair(value_converter_->getMinTSD(),
                            value_converter_->getMinWeight());
    }

    void TSDF2D::GrowLimits(const Eigen::Vector2f &point)
    {
      Grid2D::GrowLimits(point,
                         {mutable_correspondence_cost_cells(), &weight_cells_},
                         {value_converter_->getUnknownTSDValue(),
                          value_converter_->getUnknownWeightValue()});
    }

    std::unique_ptr<Grid2D> TSDF2D::ComputeCroppedGrid() const
    {
      Eigen::Array2i offset;
      CellLimits cell_limits;
      ComputeCroppedLimits(&offset, &cell_limits);
      const double resolution = limits().resolution();
      const Eigen::Vector2d max =
          limits().max() - resolution * Eigen::Vector2d(offset.y(), offset.x());
      std::unique_ptr<TSDF2D> cropped_grid = std::make_unique<TSDF2D>(MapLimits(resolution, max, cell_limits), value_converter_->getMaxTSD(), value_converter_->getMaxWeight(), conversion_tables_);
      for (const Eigen::Array2i &xy_index : XYIndexRangeIterator(cell_limits))
      {
        if (!IsKnown(xy_index + offset))
          continue;
        cropped_grid->SetCell(xy_index, GetTSD(xy_index + offset),
                              GetWeight(xy_index + offset));
      }
      cropped_grid->FinishUpdate();
      return std::move(cropped_grid);
    }

    bool TSDF2D::DrawToSubmapTexture(io::SubmapTextureData *const texture,
                                     common::Rigid3d local_pose) const
    {
      Eigen::Array2i offset;
      CellLimits cell_limits;
      ComputeCroppedLimits(&offset, &cell_limits);

      std::string cells;
      for (const Eigen::Array2i &xy_index : XYIndexRangeIterator(cell_limits))
      {
        if (!IsKnown(xy_index + offset))
        {
          cells.push_back(0); // value
          cells.push_back(0); // alpha
          continue;
        }
        // We would like to add 'delta' but this is not possible using a value and
        // alpha. We use premultiplied alpha, so when 'delta' is positive we can
        // add it by setting 'alpha' to zero. If it is negative, we set 'value' to
        // zero, and use 'alpha' to subtract. This is only correct when the pixel
        // is currently white, so walls will look too gray. This should be hard to
        // detect visually for the user, though.
        float normalized_tsdf = std::abs(GetTSD(xy_index + offset));
        normalized_tsdf =
            std::pow(normalized_tsdf / value_converter_->getMaxTSD(), 0.5f);
        float normalized_weight =
            GetWeight(xy_index + offset) / value_converter_->getMaxWeight();
        const int delta = static_cast<int>(
            std::round(normalized_weight * (normalized_tsdf * 255. - 128.)));
        const uint8 alpha = delta > 0 ? 0 : -delta;
        const uint8 value = delta > 0 ? delta : 0;
        cells.push_back(value);
        cells.push_back((value || alpha) ? alpha : 1);
      }

      std::string g_cells;
      io::FastGzipString(cells, &g_cells);
      texture->cells = g_cells;
      texture->width = cell_limits.num_x_cells;
      texture->height = cell_limits.num_y_cells;
      const double resolution = limits().resolution();
      texture->resolution = resolution;
      const double max_x = limits().max().x() - resolution * offset.y();
      const double max_y = limits().max().y() - resolution * offset.x();
      texture->slice_pose = local_pose.inverse() *
          common::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.));

      return true;
    }

  } // namespace estimator
} // namespace carto_slam
