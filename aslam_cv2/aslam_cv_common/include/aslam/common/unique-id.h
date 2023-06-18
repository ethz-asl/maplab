#ifndef ASLAM_COMMON_UNIQUE_ID_H_
#define ASLAM_COMMON_UNIQUE_ID_H_

#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <glog/logging.h>

#include <aslam/common/hash-id.h>
#include <aslam/common/internal/unique-id.h>

static constexpr unsigned kDefaultIDPrintLength = 10;

namespace aslam {
template <typename IdType>
class UniqueId;
}

#define UNIQUE_ID_DEFINE_ID(TypeName)                           \
  class TypeName : public aslam::UniqueId<TypeName> {          \
   public: /* NOLINT */                                         \
    TypeName() = default;                                       \
    explicit inline TypeName(const aslam::proto::Id& id_field) \
        : aslam::UniqueId<TypeName>(id_field) {}               \
  };                                                            \
  typedef std::vector<TypeName> TypeName##List;                 \
  typedef std::unordered_set<TypeName> TypeName##Set;           \
  extern void defineId##__FILE__##__LINE__(void)

#define UNIQUE_ID_DEFINE_IMMUTABLE_ID(TypeName, BaseTypeName)         \
  class TypeName : public aslam::UniqueId<TypeName> {                \
   public: /* NOLINT */                                               \
    TypeName() = default;                                             \
    explicit inline TypeName(const aslam::proto::Id& id_field)       \
        : aslam::UniqueId<TypeName>(id_field) {}                     \
    inline void from##BaseTypeName(const BaseTypeName& landmark_id) { \
      aslam::HashId hash_id;                                          \
      landmark_id.toHashId(&hash_id);                                 \
      this->fromHashId(hash_id);                                      \
    }                                                                 \
  };                                                                  \
  typedef std::vector<TypeName> TypeName##List;                       \
  typedef std::unordered_set<TypeName> TypeName##Set;                 \
  extern void defineId##__FILE__##__LINE__(void)

// This macro needs to be called outside of any namespace.
#define UNIQUE_ID_DEFINE_ID_HASH(TypeName)                                   \
  namespace std {                                                            \
  template <>                                                                \
  struct hash<TypeName> {                                                    \
    typedef TypeName argument_type;                                          \
    typedef std::size_t value_type;                                          \
    value_type operator()(const argument_type& hash_id) const {              \
      return hash_id.hashToSizeT();                                          \
    }                                                                        \
  };                                                                         \
                                                                             \
  template <>                                                                \
  struct hash<std::pair<TypeName, TypeName>> {                               \
    typedef TypeName argument_type;                                          \
    typedef std::size_t value_type;                                          \
    value_type operator()(                                                   \
        const std::pair<argument_type, argument_type>& hash_id_pair) const { \
      return hash_id_pair.first.hashToSizeT() ^                              \
             hash_id_pair.second.hashToSizeT();                              \
    }                                                                        \
  };                                                                         \
  }                                                                          \
  extern void defineId##__FILE__##__LINE__(void)

#define ALIAS_UNIQUE_ID(BaseIdType, AliasedIdType)  \
  typedef BaseIdType AliasedIdType;                       \
  typedef BaseIdType##List AliasedIdType##List;           \
  typedef BaseIdType##Set AliasedIdType##Set

template <typename ContainerType>
inline std::string printIdContainer(const ContainerType& container) {
  std::stringstream ss;
  ss << "[ ";
  for (const typename ContainerType::value_type& element : container) {
    ss << element << " ";
  }
  ss << "]";
  return ss.str();
}

namespace aslam {
template <typename IdType>
void generateId(IdType* id) {
  CHECK_NOTNULL(id);
  uint64_t hash[2];
  internal::generateUnique128BitHash(hash);
  id->fromUint64(hash);
}

template <typename IdType>
IdType createRandomId() {
  IdType id;
  generateId(&id);
  return id;
}

template <typename IdType>
void generateIdFromInt(unsigned int idx, IdType* id) {
  CHECK_NOTNULL(id);
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(32) << idx;
  id->fromHexString(ss.str());
}

// For internal use only.
class Id : public HashId {
 public:
  Id() = default;
  explicit inline Id(const aslam::proto::Id& id_field) {
    deserialize(id_field);
  }
  inline void deserialize(const aslam::proto::Id& id_field) {
    CHECK_EQ(id_field.uint_size(), 2);
    fromUint64(id_field.uint().data());
  }
  inline void serialize(aslam::proto::Id* id_field) const {
    CHECK_NOTNULL(id_field)->clear_uint();

    google::protobuf::RepeatedField<google::protobuf::uint64>* uint_field =
        id_field->mutable_uint();
    uint_field->Add();
    uint_field->Add();
    toUint64(uint_field->mutable_data());
  }

  inline void fromHashId(const aslam::HashId& id) {
    static_cast<aslam::HashId&>(*this) = id;
  }
  inline void toHashId(aslam::HashId* id) const {
    CHECK_NOTNULL(id);
    *id = static_cast<const aslam::HashId&>(*this);
  }

  std::string printString() const {
    constexpr int kPartLength = (kDefaultIDPrintLength - 2) / 2;
    const std::string hex = hexString();
    return hex.substr(0, kPartLength) + ".." +
           hex.substr(hex.length() - kPartLength, kPartLength);
  }

  template <typename IdType>
  inline IdType toIdType() const {
    IdType value;
    value.fromHashId(*this);
    return value;
  }

  template <typename GenerateIdType>
  friend void generateId(GenerateIdType* id);

  bool correspondsTo(const aslam::proto::Id& proto_id) const {
    Id corresponding(proto_id);
    return operator==(corresponding);
  }

 private:
  using aslam::HashId::fromUint64;
  using aslam::HashId::toUint64;
};

typedef std::unordered_set<Id> IdSet;
typedef std::vector<Id> IdList;

// To be used for general IDs.
template <typename IdType>
class UniqueId : private Id {
 public:
  UniqueId() = default;
  explicit inline UniqueId(const aslam::proto::Id& id_field) : Id(id_field) {}

  using aslam::HashId::hexString;
  using aslam::HashId::fromHexString;
  using aslam::HashId::hashToSizeT;
  using aslam::HashId::isValid;
  using aslam::HashId::setInvalid;
  using aslam::HashId::shortHex;

  using Id::deserialize;
  using Id::printString;
  using Id::serialize;
  using Id::toIdType;

  std::ostream& operator<<(std::ostream& os) const {
    return os << printString();
  }

  inline void fromHashId(const aslam::HashId& id) {
    static_cast<aslam::HashId&>(*this) = id;
  }

  inline void toHashId(aslam::HashId* id) const {
    CHECK_NOTNULL(id);
    *id = static_cast<const aslam::HashId&>(*this);
  }

  inline bool operator==(const IdType& other) const {
    return aslam::HashId::operator==(other);
  }

  inline bool operator==(const Id& other) const {
    return aslam::HashId::operator==(other);
  }

  inline bool operator!=(const IdType& other) const {
    return aslam::HashId::operator!=(other);
  }

  inline bool operator!=(const Id& other) const {
    return aslam::HashId::operator!=(other);
  }

  inline bool operator<(const IdType& other) const {
    return aslam::HashId::operator<(other);
  }

  inline bool operator<(const Id& other) const {
    return aslam::HashId::operator<(other);
  }

  template <typename GenerateIdType>
  friend void generateId(GenerateIdType* id);
};

UNIQUE_ID_DEFINE_ID(FrameId);
UNIQUE_ID_DEFINE_ID(NFramesId);
UNIQUE_ID_DEFINE_ID(SensorId);

// Maintain compatibility with old naming scheme.
ALIAS_UNIQUE_ID(SensorId, CameraId);
ALIAS_UNIQUE_ID(SensorId, NCameraId);

}  // namespace aslam

namespace std {
inline ostream& operator<<(ostream& out, const aslam::Id& hash) {
  out << hash.printString();
  return out;
}

template <typename IdType>
inline ostream& operator<<(ostream& out, const aslam::UniqueId<IdType>& hash) {
  out << hash.printString();
  return out;
}

template <>
struct hash<aslam::Id> {
  std::size_t operator()(const aslam::Id& hashId) const {
    return std::hash<std::string>()(hashId.hexString());
  }
};
}  // namespace std

UNIQUE_ID_DEFINE_ID_HASH(aslam::FrameId);
UNIQUE_ID_DEFINE_ID_HASH(aslam::NFramesId);
UNIQUE_ID_DEFINE_ID_HASH(aslam::SensorId);

#endif  // ASLAM_COMMON_UNIQUE_ID_H_
