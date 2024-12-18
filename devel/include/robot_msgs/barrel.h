// Generated by gencpp from file robot_msgs/barrel.msg
// DO NOT EDIT!


#ifndef ROBOT_MSGS_MESSAGE_BARREL_H
#define ROBOT_MSGS_MESSAGE_BARREL_H


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
struct barrel_
{
  typedef barrel_<ContainerAllocator> Type;

  barrel_()
    : header()
    , num(0)
    , id(0)
    , grade(0)
    , yaw(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  barrel_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , num(0)
    , id(0)
    , grade(0)
    , yaw(0.0)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int8_t _num_type;
  _num_type num;

   typedef int8_t _id_type;
  _id_type id;

   typedef int32_t _grade_type;
  _grade_type grade;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::robot_msgs::barrel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_msgs::barrel_<ContainerAllocator> const> ConstPtr;

}; // struct barrel_

typedef ::robot_msgs::barrel_<std::allocator<void> > barrel;

typedef boost::shared_ptr< ::robot_msgs::barrel > barrelPtr;
typedef boost::shared_ptr< ::robot_msgs::barrel const> barrelConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_msgs::barrel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_msgs::barrel_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_msgs::barrel_<ContainerAllocator1> & lhs, const ::robot_msgs::barrel_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.num == rhs.num &&
    lhs.id == rhs.id &&
    lhs.grade == rhs.grade &&
    lhs.yaw == rhs.yaw &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_msgs::barrel_<ContainerAllocator1> & lhs, const ::robot_msgs::barrel_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::barrel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::barrel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::barrel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::barrel_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::barrel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::barrel_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_msgs::barrel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "63bd136c4dab05090a6031f8df03e306";
  }

  static const char* value(const ::robot_msgs::barrel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x63bd136c4dab0509ULL;
  static const uint64_t static_value2 = 0x0a6031f8df03e306ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_msgs::barrel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_msgs/barrel";
  }

  static const char* value(const ::robot_msgs::barrel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_msgs::barrel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"int8 num\n"
"int8 id\n"
"int32 grade\n"
"float32 yaw\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
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

  static const char* value(const ::robot_msgs::barrel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_msgs::barrel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.num);
      stream.next(m.id);
      stream.next(m.grade);
      stream.next(m.yaw);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct barrel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_msgs::barrel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_msgs::barrel_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "num: ";
    Printer<int8_t>::stream(s, indent + "  ", v.num);
    s << indent << "id: ";
    Printer<int8_t>::stream(s, indent + "  ", v.id);
    s << indent << "grade: ";
    Printer<int32_t>::stream(s, indent + "  ", v.grade);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MSGS_MESSAGE_BARREL_H
