#include "ui.h"
#include <imgui.h>
#include <glm/glm.hpp>
#include <cstdio>
#include <cfloat>

namespace robotsim {

void drawUI(UIState& ui, Arm& arm, KeyframeSequence& seq) {
    ImGui::SetNextWindowSize(ImVec2(400, 640), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
    ImGui::Begin("Arm Control");

    ImGui::TextUnformatted("Mode");
    if (ImGui::RadioButton("Manual", ui.mode == Mode::Manual)) ui.mode = Mode::Manual;
    ImGui::SameLine();
    if (ImGui::RadioButton("IK", ui.mode == Mode::IK)) ui.mode = Mode::IK;
    ImGui::SameLine();
    if (ImGui::RadioButton("Playback", ui.mode == Mode::Playback)) ui.mode = Mode::Playback;
    ImGui::Separator();

    ImGui::TextUnformatted("Joints (click a name to rename; expand for range of motion)");
    bool edited = false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        ImGui::PushID(i);
        float lo = glm::degrees(arm.config(i).lower);
        float hi = glm::degrees(arm.config(i).upper);

        // Name: a button that becomes an inline text field while being renamed.
        const float name_width = 120.0f;
        if (ui.editingName == i) {
            ImGui::SetNextItemWidth(name_width);
            if (ui.nameFocusPending) {
                ImGui::SetKeyboardFocusHere();
                ui.nameFocusPending = false;
            }
            bool commit = ImGui::InputText("##name", ui.nameBuf, sizeof(ui.nameBuf),
                                           ImGuiInputTextFlags_EnterReturnsTrue);
            if (commit || ImGui::IsItemDeactivated()) {
                if (ui.nameBuf[0] != '\0') arm.setName(i, ui.nameBuf);
                ui.editingName = -1;
            }
        } else {
            if (ImGui::Button(arm.name(i).c_str(), ImVec2(name_width, 0))) {
                ui.editingName = i;
                ui.nameFocusPending = true;
                std::snprintf(ui.nameBuf, sizeof(ui.nameBuf), "%s", arm.name(i).c_str());
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Click to rename");
        }

        // Angle slider fills the rest of the row.
        ImGui::SameLine();
        float deg = glm::degrees(arm.getAngle(i));
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::SliderFloat("##deg", &deg, lo, hi, "%.1f deg")) {
            arm.setAngle(i, glm::radians(deg));
            edited = true;
        }

        // Per-joint range-of-motion dropdown.
        if (ImGui::TreeNode("range of motion")) {
            float lower = lo, upper = hi;
            bool range_edited = false;
            ImGui::SetNextItemWidth(150);
            if (ImGui::DragFloat("min (deg)", &lower, 0.5f, -360.0f, upper, "%.1f")) range_edited = true;
            ImGui::SetNextItemWidth(150);
            if (ImGui::DragFloat("max (deg)", &upper, 0.5f, lower, 360.0f, "%.1f")) range_edited = true;
            if (range_edited) arm.setLimits(i, glm::radians(lower), glm::radians(upper));
            ImGui::TreePop();
        }
        ImGui::PopID();
    }
    if (edited && ui.mode != Mode::Manual) {
        seq.playing = false;
        ui.mode = Mode::Manual;  // dragging a slider kicks us back to manual mode
    }
    if (ImGui::Button("Zero all joints")) {
        for (int i = 0; i < NUM_JOINTS; ++i) arm.setAngle(i, 0.0f);
    }

    if (ui.mode == Mode::IK) {
        ImGui::Separator();
        ImGui::TextUnformatted("IK target");
        ImGui::TextWrapped("Drag the gizmo on the tip: center = free move, "
                           "axes/planes = constrained move, rings = rotate the tip. "
                           "Orbit the camera to work from another angle.");
        ImGui::DragFloat3("target xyz", &ui.ikTarget.x, 0.005f, -5.0f, 5.0f, "%.3f");
        if (ImGui::Button("Reset target to tip")) {
            glm::mat4 ee = arm.endEffector();
            ui.ikTarget = glm::vec3(ee[3]);
            ui.ikRot    = glm::quat_cast(glm::mat3(ee));
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset orientation")) {
            ui.ikRot = glm::quat_cast(glm::mat3(arm.endEffector()));
        }
        ImGui::Text("pos error: %.3f m%s   rot error: %.1f deg",
                    ui.ikError, ui.ikError < 0.01f ? " (reached)" : "",
                    glm::degrees(ui.ikAngError));
    }

    ImGui::Separator();
    ImGui::Text("Keyframes (%zu)", seq.frames.size());

    if (ImGui::Button("Add (snapshot current pose)")) {
        Keyframe k;
        k.angles = arm.getAngles();
        k.duration_to_next = 1.0f;
        seq.frames.push_back(k);
        ui.selectedFrame = (int)seq.frames.size() - 1;
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear all")) {
        seq.frames.clear();
        seq.playing = false;
        seq.time = 0.0f;
        ui.selectedFrame = -1;
    }

    if (ImGui::BeginListBox("##frames", ImVec2(-FLT_MIN, 6 * ImGui::GetTextLineHeightWithSpacing()))) {
        for (int i = 0; i < (int)seq.frames.size(); ++i) {
            bool isLast = (i == (int)seq.frames.size() - 1);
            const char* dest = (isLast && seq.looping) ? "loop"
                             : (isLast)                ? "unused"
                                                       : "next";
            char label[80];
            std::snprintf(label, sizeof(label), "Frame %d   (%.2fs to %s)",
                          i, seq.frames[i].duration_to_next, dest);
            if (ImGui::Selectable(label, ui.selectedFrame == i)) {
                ui.selectedFrame = i;
            }
        }
        ImGui::EndListBox();
    }

    if (ui.selectedFrame >= 0 && ui.selectedFrame < (int)seq.frames.size()) {
        auto& f = seq.frames[ui.selectedFrame];
        bool isLast = (ui.selectedFrame == (int)seq.frames.size() - 1);
        const char* slider_label = (isLast && seq.looping) ? "duration to loop (s)"
                                 : (isLast)                ? "duration (unused, enable Loop)"
                                                           : "duration to next (s)";
        ImGui::SliderFloat(slider_label, &f.duration_to_next, 0.05f, 10.0f, "%.2fs");
        if (ImGui::Button("Load into sliders")) arm.setAngles(f.angles);
        ImGui::SameLine();
        if (ImGui::Button("Overwrite from current")) f.angles = arm.getAngles();
        ImGui::SameLine();
        if (ImGui::Button("Delete")) {
            seq.frames.erase(seq.frames.begin() + ui.selectedFrame);
            if (ui.selectedFrame >= (int)seq.frames.size())
                ui.selectedFrame = (int)seq.frames.size() - 1;
        }
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Playback");
    float dur = seq.totalDuration();
    if (ImGui::Button(seq.playing ? "Pause" : "Play")) {
        if (seq.frames.size() >= 2) {
            seq.playing = !seq.playing;
            ui.mode = Mode::Playback;
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop")) {
        seq.playing = false;
        seq.time = 0.0f;
    }
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &seq.looping);
    float t = seq.time;
    if (ImGui::SliderFloat("time", &t, 0.0f, dur > 0.0f ? dur : 1.0f, "%.2fs")) {
        seq.time = t;
        ui.mode = Mode::Playback;
    }
    ImGui::Text("Total duration: %.2fs", dur);

    ImGui::Separator();
    ImGui::TextUnformatted("Camera: LMB orbit, RMB/MMB pan, wheel zoom. ESC to quit.");

    ImGui::End();
}

}  // namespace robotsim
