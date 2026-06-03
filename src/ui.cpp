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
    if (ImGui::RadioButton("Manual", !ui.useKeyframes)) ui.useKeyframes = false;
    ImGui::SameLine();
    if (ImGui::RadioButton("Playback", ui.useKeyframes)) ui.useKeyframes = true;
    ImGui::Separator();

    ImGui::TextUnformatted("Joints (degrees)");
    bool edited = false;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        char label[16];
        std::snprintf(label, sizeof(label), "J%d", i + 1);
        float deg = glm::degrees(arm.getAngle(i));
        float lo  = glm::degrees(arm.config(i).lower);
        float hi  = glm::degrees(arm.config(i).upper);
        if (ImGui::SliderFloat(label, &deg, lo, hi, "%.1f deg")) {
            arm.setAngle(i, glm::radians(deg));
            edited = true;
        }
    }
    if (edited && ui.useKeyframes) {
        seq.playing = false;
        ui.useKeyframes = false;  // editing kicks us back to manual mode
    }
    if (ImGui::Button("Zero all joints")) {
        for (int i = 0; i < NUM_JOINTS; ++i) arm.setAngle(i, 0.0f);
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
            ui.useKeyframes = true;
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
        ui.useKeyframes = true;
    }
    ImGui::Text("Total duration: %.2fs", dur);

    ImGui::Separator();
    ImGui::TextUnformatted("Camera: LMB orbit, RMB/MMB pan, wheel zoom. ESC to quit.");

    ImGui::End();
}

}  // namespace robotsim
