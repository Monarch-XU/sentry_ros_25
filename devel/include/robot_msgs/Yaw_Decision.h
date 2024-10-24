// Generated by gencpp from file robot_msgs/Yaw_Decision.msg
// DO NOT EDIT!


#ifndef ROBOT_MSGS_MESSAGE_YAW_DECISION_H
#define ROBOT_MSGS_MESSAGE_YAW_DECISION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace robot_msgs
{
template <class ContainerAllocator>
struct Yaw_Decision_
{
  typedef Yaw_Decision_<ContainerAllocator> Type;

  Yaw_Decision_()
    : header()
    , yaw(0.0)
    , target_lock(0)  {
    }
  Yaw_Decision_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , yaw(0.0)
    , target_lock(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef int8_t _target_lock_type;
  _target_lock_type target_lock;





  typedef boost::shared_ptr< ::robot_msgs::Yaw_Decision_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_msgs::Yaw_Decision_<ContainerAllocator> const> ConstPtr;

}; // struct Yaw_Decision_

typedef ::robot_msgs::Yaw_Decision_<std::allocator<void> > Yaw_Decision;

typedef boost::shared_ptr< ::robot_msgs::Yaw_Decision > Yaw_DecisionPtr;
typedef boost::shared_ptr< ::robot_msgs::Yaw_Decision const> Yaw_DecisionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_msgs::Yaw_Decision_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_msgs::Yaw_Decision_<ContainerAllocator1> & lhs, const ::robot_msgs::Yaw_Decision_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.yaw == rhs.yaw &&
    lhs.target_lock == rhs.target_lock;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_msgs::Yaw_Decision_<ContainerAllocator1> & lhs, const ::robot_msgs::Yaw_Decision_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::Yaw_Decision_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::Yaw_Decision_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::Yaw_Decision_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c65cfdbaa0bdce5faf6cf2f74ce61b80";
  }

  static const char* value(const ::robot_msgs::Yaw_Decision_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc65cfdbaa0bdce5fULL;
  static const uint64_t static_value2 = 0xaf6cf2f74ce61b80ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_msgs/Yaw_Decision";
  }

  static const char* value(const ::robot_msgs::Yaw_Decision_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float32 yaw\n"
"int8 target_lock\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::robot_msgs::Yaw_Decision_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.yaw);
      stream.next(m.target_lock);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Yaw_Decision_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_msgs::Yaw_Decision_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_msgs::Yaw_Decision_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "target_lock: ";
    Printer<int8_t>::stream(s, indent + "  ", v.target_lock);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MSGS_MESSAGE_YAW_DECISION_H
