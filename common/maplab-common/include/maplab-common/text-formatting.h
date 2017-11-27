#ifndef MAPLAB_COMMON_TEXT_FORMATTING_H_
#define MAPLAB_COMMON_TEXT_FORMATTING_H_

#include <string>

namespace common {

// See http://misc.flogisoft.com/bash/tip_colors_and_formatting for a list of
// format options.
enum class FormatOptions {
  kDefault = 0,
  kBold = 1,
  kDim = 2,
  kUnderlined = 4,
  kReverse = 7,  // Invert foreground and background colours.
  kHidden = 8
};

enum class ForegroundColors {
  kDefault = 39,
  kBlack = 30,
  kRed,
  kGreen,
  kYellow,
  kBlue,
  kMagenta,
  kCyan,
  kLightGray,
  kDarkGray = 90,
  kLighRed,
  kLightGreen,
  kLightYellow,
  kLightBlue,
  kLightMagenta,
  kLightCyan,
  kWhite
};

enum class BackgroundColors {
  kDefault = 49,
  kBlack = 40,
  kRed,
  kGreen,
  kYellow,
  kBlue,
  kMagenta,
  kCyan,
  kLightGray,
  kDarkGray = 100,
  kLighRed,
  kLightGreen,
  kLightYellow,
  kLightBlue,
  kLightMagenta,
  kLightCyan,
  kWhite
};

// TODO(eggerk): support multiple format options simultaneously.
inline const std::string formatText(
    const std::string& text, const FormatOptions& format_option,
    const ForegroundColors& foreground_color,
    const BackgroundColors& background_color,
    const bool add_readline_escape_characters) {
  // Need \001 and \002 to tell readline() to ignore formatting commands.
  // See http://wiki.hackzine.org/development/misc/readline-color-prompt.html
  const std::string readline_escape_begin =
      add_readline_escape_characters ? "\001" : "";
  const std::string readling_escape_end =
      add_readline_escape_characters ? "\002" : "";
  return readline_escape_begin + "\e[" +
         std::to_string(static_cast<int>(format_option)) + ";" +
         std::to_string(static_cast<int>(foreground_color)) + ";" +
         std::to_string(static_cast<int>(background_color)) + "m" +
         readling_escape_end + text + readline_escape_begin + "\e[0m" +
         readling_escape_end;
}

inline const std::string formatText(
    const std::string& text, const FormatOptions& format_option,
    const ForegroundColors& foreground_color,
    const BackgroundColors& background_color) {
  constexpr bool kAddReadlineEscapeCharacters = false;
  return formatText(
      text, format_option, foreground_color, background_color,
      kAddReadlineEscapeCharacters);
}

inline const std::string formatText(
    const std::string& text, const FormatOptions& format_option,
    const ForegroundColors& foreground_color) {
  return formatText(
      text, format_option, foreground_color, BackgroundColors::kDefault);
}

inline const std::string formatText(
    const std::string& text, const FormatOptions& format_option) {
  return formatText(
      text, format_option, ForegroundColors::kDefault,
      BackgroundColors::kDefault);
}

inline const std::string colorText(
    const std::string& text, const ForegroundColors& foreground_color) {
  return formatText(
      text, FormatOptions::kDefault, foreground_color,
      BackgroundColors::kDefault);
}

inline const std::string colorText(
    const std::string& text, const ForegroundColors& foreground_color,
    const BackgroundColors& background_color) {
  return formatText(
      text, FormatOptions::kDefault, foreground_color, background_color);
}

}  // namespace common

#endif  // MAPLAB_COMMON_TEXT_FORMATTING_H_
