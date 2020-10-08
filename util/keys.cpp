#include "keys.h"

namespace utils {
namespace keys {

const char* name( vl::EKey key ) {
    switch ( key ) {
        case vl::Key_0: return "0";
        case vl::Key_1: return "1";
        case vl::Key_2: return "2";
        case vl::Key_3: return "3";
        case vl::Key_4: return "4";
        case vl::Key_5: return "5";
        case vl::Key_6: return "6";
        case vl::Key_7: return "7";
        case vl::Key_8: return "8";
        case vl::Key_9: return "9";

        case vl::Key_A: return "A";
        case vl::Key_B: return "B";
        case vl::Key_C: return "C";
        case vl::Key_D: return "D";
        case vl::Key_E: return "E";
        case vl::Key_F: return "F";
        case vl::Key_G: return "G";
        case vl::Key_H: return "H";
        case vl::Key_I: return "I";
        case vl::Key_J: return "J";
        case vl::Key_K: return "K";
        case vl::Key_L: return "L";
        case vl::Key_M: return "M";
        case vl::Key_N: return "N";
        case vl::Key_O: return "O";
        case vl::Key_P: return "P";
        case vl::Key_Q: return "Q";
        case vl::Key_R: return "R";
        case vl::Key_S: return "S";
        case vl::Key_T: return "T";
        case vl::Key_U: return "U";
        case vl::Key_V: return "V";
        case vl::Key_W: return "W";
        case vl::Key_X: return "X";
        case vl::Key_Y: return "Y";
        case vl::Key_Z: return "Z";

        case vl::Key_Return: return "Rtn";
        case vl::Key_BackSpace: return "Bsp";
        case vl::Key_Tab: return "Tab";
        case vl::Key_Space: return "Spc";

        case vl::Key_Clear: return "Clr";
        case vl::Key_Escape: return "Esc";
        case vl::Key_Exclam: return "!";
        case vl::Key_QuoteDbl: return "\"";
        case vl::Key_Hash: return "#";
        case vl::Key_Dollar: return "$";
        case vl::Key_Ampersand: return "&";
        case vl::Key_Quote: return "'";
        case vl::Key_LeftParen: return "(";
        case vl::Key_RightParen: return ")";
        case vl::Key_Asterisk: return "*";
        case vl::Key_Plus: return "+";
        case vl::Key_Comma: return ",";
        case vl::Key_Minus: return "-";
        case vl::Key_Period: return ".";
        case vl::Key_Slash: return "/";
        case vl::Key_Colon: return ":";
        case vl::Key_Semicolon: return ";";
        case vl::Key_Less: return "<";
        case vl::Key_Equal: return "=";
        case vl::Key_Greater: return ">";
        case vl::Key_Question: return "?";
        case vl::Key_At: return "@";
        case vl::Key_LeftBracket: return "[";
        case vl::Key_BackSlash: return "\\";
        case vl::Key_RightBracket: return "]";
        case vl::Key_Caret: return "^";
        case vl::Key_Underscore: return "_";
        case vl::Key_QuoteLeft: return "`";

// non unicode keys

        case vl::Key_Ctrl: return "Ctrl";
        case vl::Key_LeftCtrl: return "LCtrl";
        case vl::Key_RightCtrl: return "RCtrl";
        case vl::Key_Alt: return "Alt";
        case vl::Key_LeftAlt: return "LAlt";
        case vl::Key_RightAlt: return "RAlt";
        case vl::Key_Shift: return "Shft";
        case vl::Key_LeftShift: return "LSft";
        case vl::Key_RightShift: return "RSft";
        case vl::Key_Insert: return "Ins";
        case vl::Key_Delete: return "Del";
        case vl::Key_Home: return "Hom";
        case vl::Key_End: return "End";
        case vl::Key_Print: return "Prt";
        case vl::Key_Pause: return "Paus";
        case vl::Key_PageUp: return "PgUp";
        case vl::Key_PageDown: return "PgDn";
        case vl::Key_Left: return "Lft";
        case vl::Key_Right: return "Rgt";
        case vl::Key_Up: return "Up";
        case vl::Key_Down: return "Dwn";
        case vl::Key_F1: return "F1";
        case vl::Key_F2: return "F2";
        case vl::Key_F3: return "F3";
        case vl::Key_F4: return "F4";
        case vl::Key_F5: return "F5";
        case vl::Key_F6: return "F6";
        case vl::Key_F7: return "F7";
        case vl::Key_F8: return "F8";
        case vl::Key_F9: return "F9";
        case vl::Key_F10: return "F10";
        case vl::Key_F11: return "F11";
        case vl::Key_F12: return "F12";
        default: return "InvalidKey";
    }
}

}
}