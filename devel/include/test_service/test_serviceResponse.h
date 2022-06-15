// Generated by gencpp from file test_service/test_serviceResponse.msg
// DO NOT EDIT!


#ifndef TEST_SERVICE_MESSAGE_TEST_SERVICERESPONSE_H
#define TEST_SERVICE_MESSAGE_TEST_SERVICERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace test_service
{
template <class ContainerAllocator>
struct test_serviceResponse_
{
  typedef test_serviceResponse_<ContainerAllocator> Type;

  test_serviceResponse_()
    : square(0.0)  {
    }
  test_serviceResponse_(const ContainerAllocator& _alloc)
    : square(0.0)  {
  (void)_alloc;
    }



   typedef double _square_type;
  _square_type square;





  typedef boost::shared_ptr< ::test_service::test_serviceResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::test_service::test_serviceResponse_<ContainerAllocator> const> ConstPtr;

}; // struct test_serviceResponse_

typedef ::test_service::test_serviceResponse_<std::allocator<void> > test_serviceResponse;

typedef boost::shared_ptr< ::test_service::test_serviceResponse > test_serviceResponsePtr;
typedef boost::shared_ptr< ::test_service::test_serviceResponse const> test_serviceResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test_service::test_serviceResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::test_service::test_serviceResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::test_service::test_serviceResponse_<ContainerAllocator1> & lhs, const ::test_service::test_serviceResponse_<ContainerAllocator2> & rhs)
{
  return lhs.square == rhs.square;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::test_service::test_serviceResponse_<ContainerAllocator1> & lhs, const ::test_service::test_serviceResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace test_service

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::test_service::test_serviceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test_service::test_serviceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_service::test_serviceResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test_service::test_serviceResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_service::test_serviceResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test_service::test_serviceResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test_service::test_serviceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0407b898ade723d5e53383fd7f09e8ec";
  }

  static const char* value(const ::test_service::test_serviceResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0407b898ade723d5ULL;
  static const uint64_t static_value2 = 0xe53383fd7f09e8ecULL;
};

template<class ContainerAllocator>
struct DataType< ::test_service::test_serviceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test_service/test_serviceResponse";
  }

  static const char* value(const ::test_service::test_serviceResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test_service::test_serviceResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 square\n"
"\n"
;
  }

  static const char* value(const ::test_service::test_serviceResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test_service::test_serviceResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.square);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct test_serviceResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test_service::test_serviceResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test_service::test_serviceResponse_<ContainerAllocator>& v)
  {
    s << indent << "square: ";
    Printer<double>::stream(s, indent + "  ", v.square);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEST_SERVICE_MESSAGE_TEST_SERVICERESPONSE_H
