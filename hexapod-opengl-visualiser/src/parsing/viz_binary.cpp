#include "visualiser/parsing/viz_binary.hpp"

#include "minphys_viz_protocol.hpp"

namespace visualiser::parsing {

visualiser::scene::ShapeType ShapeFromViz(int shape) {
  switch (static_cast<minphys_viz::VizShapeType>(shape)) {
    case minphys_viz::VizShapeType::Sphere:
      return visualiser::scene::ShapeType::kSphere;
    case minphys_viz::VizShapeType::Box:
      return visualiser::scene::ShapeType::kBox;
    case minphys_viz::VizShapeType::Plane:
      return visualiser::scene::ShapeType::kPlane;
    case minphys_viz::VizShapeType::Capsule:
      return visualiser::scene::ShapeType::kCapsule;
    case minphys_viz::VizShapeType::Cylinder:
      return visualiser::scene::ShapeType::kCylinder;
    case minphys_viz::VizShapeType::HalfCylinder:
      return visualiser::scene::ShapeType::kHalfCylinder;
    case minphys_viz::VizShapeType::Compound:
      return visualiser::scene::ShapeType::kCompound;
    default:
      return visualiser::scene::ShapeType::kUnknown;
  }
}

bool ParseVizBinaryPacket(const std::uint8_t* data,
                          std::size_t size,
                          std::map<std::uint32_t, visualiser::scene::EntityState>& entities,
                          visualiser::scene::TerrainPatchState& terrain_patch,
                          std::string& packet_kind,
                          VizTerrainReassembly& terrain_reassembly) {
  (void)entities;
  (void)terrain_patch;
  (void)terrain_reassembly;
  if (size < sizeof(minphys_viz::VizWireHeader)) {
    return false;
  }
  const auto* header = reinterpret_cast<const minphys_viz::VizWireHeader*>(data);
  if (!(header->magic[0] == 'M' && header->magic[1] == 'P' && header->magic[2] == 'V' && header->magic[3] == '1')) {
    return false;
  }
  packet_kind = "viz.binary";
  return true;
}

}  // namespace visualiser::parsing
