#include "../plugin.hpp"
#include "BaxandallEQ.hpp"
#include "ClippingStage.hpp"
#include "../shared/VariableOversampling.hpp"
#include "../shared/shelf_filter.hpp"
#include <array>

struct ChowDer : Module {
    enum ParamIds {
        BASS_PARAM,
        TREBLE_PARAM,
        DRIVE_PARAM,
        BIAS_PARAM,
        BASS_MOD_DEPTH_PARAM,
        TREBLE_MOD_DEPTH_PARAM,
        DRIVE_MOD_DEPTH_PARAM,
        NUM_PARAMS
    };
    enum InputIds {
        AUDIO_IN_L,
        AUDIO_IN_R,
        BASS_MOD_INPUT,
        TREBLE_MOD_INPUT,
        DRIVE_MOD_INPUT,
        NUM_INPUTS
    };
    enum OutputIds {
        AUDIO_OUT_L,
        AUDIO_OUT_R,
        NUM_OUTPUTS
    };
    enum LightIds {
        NUM_LIGHTS
    };

    ChowDer() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        configInput(AUDIO_IN_L, "Left Audio");
        configInput(AUDIO_IN_R, "Right Audio");
        configInput(BASS_MOD_INPUT, "Bass modulation");
        configInput(TREBLE_MOD_INPUT, "Treble modulation");
        configInput(DRIVE_MOD_INPUT, "Drive modulation");
        configOutput(AUDIO_OUT_L, "Left Audio");
        configOutput(AUDIO_OUT_R, "Right Audio");
        configBypass(AUDIO_IN_L, AUDIO_OUT_L);
        configBypass(AUDIO_IN_R, AUDIO_OUT_R);

        configParam(BASS_PARAM, -1.0f, 1.0f, 0.0f, "Bass");
        configParam(TREBLE_PARAM, -1.0f, 1.0f, 0.0f, "Treble");
        configParam(DRIVE_PARAM, 0.0f, 1.0f, 0.5f, "Drive");
        configParam(BIAS_PARAM, 0.0f, 1.0f, 0.0f, "Bias");
        
        configParam(BASS_MOD_DEPTH_PARAM, -1.0f, 1.0f, 0.0f, "Bass modulation depth");
        configParam(TREBLE_MOD_DEPTH_PARAM, -1.0f, 1.0f, 0.0f, "Treble modulation depth");
        configParam(DRIVE_MOD_DEPTH_PARAM, -1.0f, 1.0f, 0.0f, "Drive modulation depth");

        // Initialize both channels
        for (int i = 0; i < 2; i++) {
            oversample[i].setOversamplingIndex(1);
        }
        onSampleRateChange();
        paramDivider.setDivision(ParamDivide);
    }

    void onSampleRateChange() override {
        float newSampleRate = getSampleRate();

        for (int i = 0; i < 2; i++) {
            oversample[i].reset(newSampleRate);
            clippers[i].reset(new ClippingStage(newSampleRate * oversample[i].getOversamplingRatio()));
            dcBlockers[i].setParameters(BiquadFilter::HIGHPASS, 30.0f / newSampleRate, M_SQRT1_2, 1.0f);
        }
        cookParams(newSampleRate);
    }

    void onReset() override {
        Module::onReset();
        onSampleRateChange();
    }

    float getModulatedParam(int paramId, int modInputId, int depthParamId) {
        float baseValue = params[paramId].getValue();
        float modValue = inputs[modInputId].getVoltage() / 5.0f;
        float depth = params[depthParamId].getValue();
        return clamp(baseValue + (modValue * depth), -1.0f, 1.0f);
    }

    void cookParams(float fs) {
        float bassValue = getModulatedParam(BASS_PARAM, BASS_MOD_INPUT, BASS_MOD_DEPTH_PARAM);
        float trebleValue = getModulatedParam(TREBLE_PARAM, TREBLE_MOD_INPUT, TREBLE_MOD_DEPTH_PARAM);
        float driveValue = getModulatedParam(DRIVE_PARAM, DRIVE_MOD_INPUT, DRIVE_MOD_DEPTH_PARAM);
        driveValue = clamp(driveValue, 0.0f, 1.0f);

        auto lowGain = dsp::dbToAmplitude(bassValue * 9.0f - 20.0f);
        auto highGain = dsp::dbToAmplitude(trebleValue * 9.0f - 20.0f);
        
        for (int i = 0; i < 2; i++) {
            shelfFilters[i].calcCoefs(lowGain, highGain, 600.0f, fs);
        }

        driveGain = dsp::dbToAmplitude(driveValue * 30.0f);
        bias = params[BIAS_PARAM].getValue() * 2.5f;
    }

    void process(const ProcessArgs& args) override {
        if(paramDivider.process())
            cookParams(args.sampleRate);

        // Process left channel
        float xL = inputs[AUDIO_IN_L].getVoltage();
        xL = driveGain * shelfFilters[0].process(xL) + bias;

        oversample[0].upsample(xL);
        float* osBufferL = oversample[0].getOSBuffer();
        for(int k = 0; k < oversample[0].getOversamplingRatio(); k++)
            osBufferL[k] = clippers[0]->processSample(osBufferL[k]);
        float yL = oversample[0].downsample();

        outputs[AUDIO_OUT_L].setVoltage(dcBlockers[0].process(yL));

        // Process right channel if connected
        if (inputs[AUDIO_IN_R].isConnected()) {
            float xR = inputs[AUDIO_IN_R].getVoltage();
            xR = driveGain * shelfFilters[1].process(xR) + bias;

            oversample[1].upsample(xR);
            float* osBufferR = oversample[1].getOSBuffer();
            for(int k = 0; k < oversample[1].getOversamplingRatio(); k++)
                osBufferR[k] = clippers[1]->processSample(osBufferR[k]);
            float yR = oversample[1].downsample();

            outputs[AUDIO_OUT_R].setVoltage(dcBlockers[1].process(yR));
        } else {
            // Copy left channel to right if no right input
            outputs[AUDIO_OUT_R].setVoltage(outputs[AUDIO_OUT_L].getVoltage());
        }
    }

    json_t* dataToJson() override {
        json_t* rootJ = json_object();
        json_object_set_new(rootJ, "osIdx", json_integer(oversample[0].getOversamplingIndex()));
        return rootJ;
    }

    void dataFromJson(json_t* rootJ) override {
        if(auto* osJson = json_object_get(rootJ, "osIdx")) {
            int osIdx = json_integer_value(osJson);
            oversample[0].setOversamplingIndex(osIdx);
            oversample[1].setOversamplingIndex(osIdx);
        }
    }

    std::array<VariableOversampling<>, 2> oversample;

private:
    enum {
        ParamDivide = 64,
    };

    float driveGain = 1.0f;
    float bias = 0.0f;
    dsp::ClockDivider paramDivider;

    // Stereo processing chains
    std::array<BiquadFilter, 2> dcBlockers;
    std::array<ShelfFilter, 2> shelfFilters;
    std::array<std::unique_ptr<ClippingStage>, 2> clippers;
};

struct ChowDerWidget : ModuleWidget {
    ChowDerWidget(ChowDer* module) {
        setModule(module);
        setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/ChowDer.svg")));

        box.size = Vec(9 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT);

        createScrews (*this);

        // Main parameters - left column
        addParam(createParamCentered<ChowKnob>(mm2px(Vec(12.0, 23.0)), module, ChowDer::BASS_PARAM));
        addParam(createParamCentered<ChowKnob>(mm2px(Vec(12.0, 43.0)), module, ChowDer::TREBLE_PARAM));
        addParam(createParamCentered<ChowKnob>(mm2px(Vec(12.0, 63.0)), module, ChowDer::DRIVE_PARAM));
        addParam(createParamCentered<ChowKnob>(mm2px(Vec(12.0, 83.0)), module, ChowDer::BIAS_PARAM));

        // Modulation depth knobs - middle column (using smaller knobs)
        addParam(createParamCentered<ChowKnob>(mm2px(Vec(22.5, 23.0)), module, ChowDer::BASS_MOD_DEPTH_PARAM));
        addParam(createParamCentered<ChowKnob>(mm2px(Vec(22.5, 43.0)), module, ChowDer::TREBLE_MOD_DEPTH_PARAM));
        addParam(createParamCentered<ChowKnob>(mm2px(Vec(22.5, 63.0)), module, ChowDer::DRIVE_MOD_DEPTH_PARAM));

        // Modulation inputs - right column
        addInput(createInputCentered<ChowPort>(mm2px(Vec(33.0, 23.0)), module, ChowDer::BASS_MOD_INPUT));
        addInput(createInputCentered<ChowPort>(mm2px(Vec(33.0, 43.0)), module, ChowDer::TREBLE_MOD_INPUT));
        addInput(createInputCentered<ChowPort>(mm2px(Vec(33.0, 63.0)), module, ChowDer::DRIVE_MOD_INPUT));

        // Stereo I/O at bottom
        addInput(createInputCentered<ChowPort>(mm2px(Vec(12.0, 110.0)), module, ChowDer::AUDIO_IN_L));
        addInput(createInputCentered<ChowPort>(mm2px(Vec(33.0, 110.0)), module, ChowDer::AUDIO_IN_R));
        addOutput(createOutputCentered<ChowPort>(mm2px(Vec(12.0, 122.0)), module, ChowDer::AUDIO_OUT_L));
        addOutput(createOutputCentered<ChowPort>(mm2px(Vec(33.0, 122.0)), module, ChowDer::AUDIO_OUT_R));
    }

    void appendContextMenu(Menu *menu) override {
        menu->addChild(new MenuSeparator());
        dynamic_cast<ChowDer*>(module)->oversample[0].addContextMenu(menu, module);
    }
};

Model* modelChowDer = createModel<ChowDer, ChowDerWidget>("ChowDer");
