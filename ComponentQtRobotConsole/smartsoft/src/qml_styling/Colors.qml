pragma Singleton
import QtQuick 2.0


QtObject{
    id: colors_defined
    readonly property color black: "#000000"
    readonly property color color_back_ground: "teal"//"#394454"

    readonly property color color_label_text: "#f0f0f0"
    readonly property color color_textfield_text: "#f0f0f0"

    //Component
    readonly property color color_component_name: "#FFFFFF"
    readonly property color color_component_bg: "#337AB7"
    readonly property color color_component_bg_border: Qt.darker(color_component_bg, 1.18)

    //States
    readonly property color color_state_on: "#c1e2b3"
    readonly property color color_state_off: "#f2dede"
    readonly property color color_state_border_on: Qt.darker(color_state_on, 1.18)
    readonly property color color_state_border_off: Qt.darker(color_state_off, 1.18)
    readonly property color color_state_zombie: "#696969" 

    //text input
    readonly property color color_text_input_text: "#000000"

    //Button
    readonly property color color_button_rectangle: "#444"
    readonly property color color_button_rectangle_border: Qt.darker(color_button_rectangle, 1.18)
    readonly property color color_button_text: "#FFFFFF"   
}
