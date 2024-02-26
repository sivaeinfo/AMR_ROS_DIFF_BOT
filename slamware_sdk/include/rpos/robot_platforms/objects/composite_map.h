#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>

#include <rpos/core/metadata.h>
#include <rpos/core/pose.h>
#include <rpos/robot_platforms/objects/map_layer.h>
#include <vector>

namespace rpos { namespace robot_platforms { namespace objects {

    class RPOS_SLAMWARE_API CompositeMap : private boost::noncopyable
    {
    public:
        CompositeMap();
        CompositeMap(const CompositeMap&);
        CompositeMap(core::Metadata metadata, std::vector< boost::shared_ptr<MapLayer> > maps);

    public:
        const core::Metadata& metadata() const;
        core::Metadata& metadata();

        const std::vector< boost::shared_ptr<MapLayer> >& maps() const;
        std::vector< boost::shared_ptr<MapLayer> >& maps(); 

        bool isMultiFloorMap() const;
        bool isMultiFloorMap(std::string& defaultMap) const;

        //get map of specific floor
        std::vector<boost::shared_ptr<MapLayer>> filterMaps(const std::map<std::string,std::string>& criteria) const;
    private:
        core::Metadata metadata_;
        std::vector< boost::shared_ptr<MapLayer> > maps_;
    };

}}}