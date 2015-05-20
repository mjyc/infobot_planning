#ifndef _INFOBOT_TOPO_MAPPING__TOPO_EXCEPTION_
#define _INFOBOT_TOPO_MAPPING__TOPO_EXCEPTION_

#include <stdexcept>
#include <string>

namespace infobot_topo_mapping
{

class TopoException: public std::runtime_error
{
 public:
  explicit TopoException(const std::string& what_arg)
      : std::runtime_error(what_arg)
  {}
};

}  // namespace infobot_topo_mapping

#endif
