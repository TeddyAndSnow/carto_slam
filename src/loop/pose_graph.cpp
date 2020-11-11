
#include "loop/pose_graph.h"

#include "loop/constraint_builder.h"
#include "loop/optimization_problem_options.h"
#include "common/transform.h"
// #include "glog/logging.h"

namespace carto_slam
{
  namespace loop
  {

    // proto::PoseGraph::Constraint::Tag ToProto(const PoseGraph::Constraint::Tag& tag) {
    //   switch (tag) {
    //     case PoseGraph::Constraint::Tag::INTRA_SUBMAP:
    //       return proto::PoseGraph::Constraint::INTRA_SUBMAP;
    //     case PoseGraph::Constraint::Tag::INTER_SUBMAP:
    //       return proto::PoseGraph::Constraint::INTER_SUBMAP;
    //   }
    //   LOG(FATAL) << "Unsupported tag.";
    // }

    // PoseGraph::Constraint::Tag FromProto(const proto::PoseGraph::Constraint::Tag& proto) {
    //   switch (proto) {
    //     case proto::PoseGraph::Constraint::INTRA_SUBMAP:
    //       return PoseGraph::Constraint::Tag::INTRA_SUBMAP;
    //     case proto::PoseGraph::Constraint::INTER_SUBMAP:
    //       return PoseGraph::Constraint::Tag::INTER_SUBMAP;
    //     case ::google::protobuf::kint32max:
    //     case ::google::protobuf::kint32min: {
    //     }
    //   }
    //   LOG(FATAL) << "Unsupported tag.";
    // }

    // std::vector<PoseGraph::Constraint> FromProto(const ::google::protobuf::RepeatedPtrField<proto::PoseGraph::Constraint>& constraint_protos) {
    //   std::vector<PoseGraph::Constraint> constraints;
    //   for (const auto& constraint_proto : constraint_protos) {
    //     const mapping::SubmapId submap_id{
    //         constraint_proto.submap_id().trajectory_id(),
    //         constraint_proto.submap_id().submap_index()};
    //     const mapping::NodeId node_id{constraint_proto.node_id().trajectory_id(),
    //                                   constraint_proto.node_id().node_index()};
    //     const PoseGraph::Constraint::Pose pose{
    //         transform::ToRigid3(constraint_proto.relative_pose()),
    //         constraint_proto.translation_weight(),
    //         constraint_proto.rotation_weight()};
    //     const PoseGraph::Constraint::Tag tag = FromProto(constraint_proto.tag());
    //     constraints.push_back(PoseGraph::Constraint{submap_id, node_id, pose, tag});
    //   }
    //   return constraints;
    // }

    // void PopulateOverlappingSubmapsTrimmerOptions2D(
    //     proto::PoseGraphOptions *const pose_graph_options,
    //     common::LuaParameterDictionary *const parameter_dictionary)
    // {
    //   constexpr char kDictionaryKey[] = "overlapping_submaps_trimmer_2d";
    //   if (!parameter_dictionary->HasKey(kDictionaryKey))
    //     return;

    //   auto options_dictionary = parameter_dictionary->GetDictionary(kDictionaryKey);
    //   auto *options = pose_graph_options->mutable_overlapping_submaps_trimmer_2d();
    //   options->set_fresh_submaps_count(
    //       options_dictionary->GetInt("fresh_submaps_count"));
    //   options->set_min_covered_area(
    //       options_dictionary->GetDouble("min_covered_area"));
    //   options->set_min_added_submaps_count(
    //       options_dictionary->GetInt("min_added_submaps_count"));
    // }

    // proto::PoseGraphOptions CreatePoseGraphOptions(
    //     common::LuaParameterDictionary *const parameter_dictionary)
    // {
    //   proto::PoseGraphOptions options;
    //   options.set_optimize_every_n_nodes(
    //       parameter_dictionary->GetInt("optimize_every_n_nodes"));
    //   *options.mutable_constraint_builder_options() =
    //       constraints::CreateConstraintBuilderOptions(
    //           parameter_dictionary->GetDictionary("constraint_builder").get());
    //   options.set_matcher_translation_weight(
    //       parameter_dictionary->GetDouble("matcher_translation_weight"));
    //   options.set_matcher_rotation_weight(
    //       parameter_dictionary->GetDouble("matcher_rotation_weight"));
    //   *options.mutable_optimization_problem_options() =
    //       optimization::CreateOptimizationProblemOptions(
    //           parameter_dictionary->GetDictionary("optimization_problem").get());
    //   options.set_max_num_final_iterations(
    //       parameter_dictionary->GetNonNegativeInt("max_num_final_iterations"));
    //   CHECK_GT(options.max_num_final_iterations(), 0);
    //   options.set_global_sampling_ratio(
    //       parameter_dictionary->GetDouble("global_sampling_ratio"));
    //   options.set_log_residual_histograms(
    //       parameter_dictionary->GetBool("log_residual_histograms"));
    //   options.set_global_constraint_search_after_n_seconds(
    //       parameter_dictionary->GetDouble(
    //           "global_constraint_search_after_n_seconds"));
    //   PopulateOverlappingSubmapsTrimmerOptions2D(&options, parameter_dictionary);
    //   return options;
    // }

  } // namespace loop
} // namespace carto_slam
