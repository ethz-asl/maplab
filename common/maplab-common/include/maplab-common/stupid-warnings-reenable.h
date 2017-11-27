// Special header guard to match stupid-warnings-disable.h
#ifndef MAPLAB_COMMON_STUPID_WARNINGS_REENABLE_H_  // For linter.
#define MAPLAB_COMMON_STUPID_WARNINGS_REENABLE_H_
#undef MAPLAB_COMMON_STUPID_WARNINGS_REENABLE_H_
#ifdef MAPLAB_WARNINGS_DISABLED
#undef MAPLAB_WARNINGS_DISABLED

#ifdef defined __clang__
#pragma clang diagnostic pop
#endif

#endif  // MAPLAB_WARNINGS_DISABLED  // NOLINT
#endif  // MAPLAB_COMMON_STUPID_WARNINGS_REENABLE_H_
