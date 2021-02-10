pragma Singleton
import QtQuick 2.0

QtObject{
    id: fonts_defined


    //General
    readonly property string font_family_titles: "fontawesome"
    readonly property string font_family_text: "fontawesome"
    readonly property int font_pixelsize_verybig: 20
    readonly property int font_pixelsize_big: 16
    readonly property int font_pixelsize_medium: 12
    readonly property int font_pixelsize_small: 8
    readonly property int font_pixelsize_very_small: 4

    //Component
    readonly property int component_name_font_size: 12
    readonly property string component_name_font_family: "fontawesome"

    //Text Input
    readonly property int font_size_text_input_size: 12
    readonly property string font_family_text_input: "fontawesome"


    //States
    readonly property int state_name_font_size: 12
    readonly property string state_name_font_family: "fontawesome"

    //Refresh button
    readonly property int refresh_button_font_size: 12
    readonly property string refresh_button_font_family: "fontawesome"
}
