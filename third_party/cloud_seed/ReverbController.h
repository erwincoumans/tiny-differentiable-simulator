//see https://github.com/ValdemarOrn/CloudSeed
//port to Daisy and App_Dekrispator by Erwin Coumans

#ifndef REVERBCONTROLLER
#define REVERBCONTROLLER

#include <vector>
#include "Default.h"
#include "Parameter.h"
#include "ReverbChannel.h"

#include "ReverbController.h"
#include "AudioLib/ValueTables.h"
#include "AllpassDiffuser.h"
#include "MultitapDiffuser.h"
#include "Utils.h"

namespace CloudSeed
{
	class ReverbController
	{
	private:
		static const int bufferSize = 16; // just make it huge by default...
		int samplerate;

		ReverbChannel channelL;
		ReverbChannel channelR;
		float leftChannelIn[bufferSize];
		float rightChannelIn[bufferSize];
		float leftLineBuffer[bufferSize];
		float rightLineBuffer[bufferSize];
		float parameters[(int)Parameter::Count];

	public:
		ReverbController(int samplerate)
			: channelL(bufferSize, samplerate, ChannelLR::Left)
			, channelR(bufferSize, samplerate, ChannelLR::Right)
		{
			this->samplerate = samplerate;
			//initFactoryChorus();
			//initFactoryDullEchos();
			//initFactoryHyperplane();
			//initFactoryMediumSpace();
			//initFactoryNoiseInTheHallway();
			initFactoryRubiKaFields();
			//initFactorySmallRoom();
			//initFactory90sAreBack();
			//initFactoryThroughTheLookingGlass();
			
		}

		void initFactoryChorus()
		{
			//parameters from Chorus Delay in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0.0;
			parameters[(int)Parameter::PreDelay] = 0.070000000298023224;
			parameters[(int)Parameter::HighPass] = 0.0;
			parameters[(int)Parameter::LowPass] = 0.29000008106231689;
			parameters[(int)Parameter::TapCount]= 0.36499997973442078;
			parameters[(int)Parameter::TapLength]= 1.0;
			parameters[(int)Parameter::TapGain] = 1.0;
			parameters[(int)Parameter::TapDecay] = 0.86500012874603271;
			parameters[(int)Parameter::DiffusionEnabled] = 1.0;
			parameters[(int)Parameter::DiffusionStages] = 0.4285714328289032;
			parameters[(int)Parameter::DiffusionDelay] = 0.43500006198883057;
			parameters[(int)Parameter::DiffusionFeedback] = 0.725000262260437;
			parameters[(int)Parameter::LineCount] = 1.0;
			parameters[(int)Parameter::LineDelay] = 0.68499988317489624;
			parameters[(int)Parameter::LineDecay] = 0.68000012636184692;
			parameters[(int)Parameter::LateDiffusionEnabled] = 1.0;
			parameters[(int)Parameter::LateDiffusionStages] = 0.28571429848670959;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.54499995708465576;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.65999996662139893;
			parameters[(int)Parameter::PostLowShelfGain] = 0.5199999213218689;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.31499990820884705;
			parameters[(int)Parameter::PostHighShelfGain] = 0.83500003814697266;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.73000013828277588;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.73499983549118042;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.50000005960464478;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.42500010132789612;
			parameters[(int)Parameter::LineModAmount] = 0.59000003337860107;
			parameters[(int)Parameter::LineModRate] = 0.46999993920326233;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.619999885559082;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.42500019073486328;
			parameters[(int)Parameter::TapSeed] = 0.0011500000255182385;
			parameters[(int)Parameter::DiffusionSeed] = 0.00018899999849963933;
			parameters[(int)Parameter::DelaySeed] = 0.00033700000494718552;
			parameters[(int)Parameter::PostDiffusionSeed] = 0.00050099997315555811;
			parameters[(int)Parameter::CrossSeed] = 0.0;
			parameters[(int)Parameter::DryOut] = 0.94499987363815308;
			parameters[(int)Parameter::PredelayOut] = 0.0;
			parameters[(int)Parameter::EarlyOut] = 0.77999997138977051;
			parameters[(int)Parameter::MainOut] = 0.74500006437301636;
			parameters[(int)Parameter::HiPassEnabled] = 0.0;
			parameters[(int)Parameter::LowPassEnabled] = 0.0;
			parameters[(int)Parameter::LowShelfEnabled] = 0.0;
			parameters[(int)Parameter::HighShelfEnabled] = 0.0;
			parameters[(int)Parameter::CutoffEnabled] = 1.0;
			parameters[(int)Parameter::LateStageTap] = 1.0;
			parameters[(int)Parameter::Interpolation] = 0.0;

			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}


		void initFactoryDullEchos()
		{
			//parameters from Dull Echos in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0.0;
			parameters[(int)Parameter::PreDelay] = 0.070000000298023224;
			parameters[(int)Parameter::HighPass] = 0.0;
			parameters[(int)Parameter::LowPass] = 0.29000008106231689;
			parameters[(int)Parameter::TapCount] = 0.36499997973442078;
			parameters[(int)Parameter::TapLength] = 1.0;
			parameters[(int)Parameter::TapGain] = 0.83499991893768311;
			parameters[(int)Parameter::TapDecay] = 0.86500012874603271;
			parameters[(int)Parameter::DiffusionEnabled] = 1.0;
			parameters[(int)Parameter::DiffusionStages] = 0.4285714328289032;
			parameters[(int)Parameter::DiffusionDelay] = 0.43500006198883057;
			parameters[(int)Parameter::DiffusionFeedback] = 0.725000262260437;
			parameters[(int)Parameter::LineCount] = 1.0;
			parameters[(int)Parameter::LineDelay] = 0.34500002861022949;
			parameters[(int)Parameter::LineDecay] = 0.41500008106231689;
			parameters[(int)Parameter::LateDiffusionEnabled] = 0.0;
			parameters[(int)Parameter::LateDiffusionStages] = 0.57142859697341919;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.66499996185302734;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.61000001430511475;
			parameters[(int)Parameter::PostLowShelfGain] = 0.5199999213218689;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.31499990820884705;
			parameters[(int)Parameter::PostHighShelfGain] = 0.83500003814697266;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.73000013828277588;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.73499983549118042;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.25499999523162842;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.3250001072883606;
			parameters[(int)Parameter::LineModAmount] = 0.33500000834465027;
			parameters[(int)Parameter::LineModRate] = 0.26999998092651367;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.13499975204467773;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.27500006556510925;
			parameters[(int)Parameter::TapSeed] = 0.0011500000255182385;
			parameters[(int)Parameter::DiffusionSeed] = 0.00018899999849963933;
			parameters[(int)Parameter::DelaySeed] = 0.0002730000123847276;
			parameters[(int)Parameter::PostDiffusionSeed] = 0.00050099997315555811;
			parameters[(int)Parameter::CrossSeed] = 0.5;
			parameters[(int)Parameter::DryOut] = 1.0;
			parameters[(int)Parameter::PredelayOut] = 0.0;
			parameters[(int)Parameter::EarlyOut] = 0.77999997138977051;
			parameters[(int)Parameter::MainOut] = 0.74500006437301636;
			parameters[(int)Parameter::HiPassEnabled] = 0.0;
			parameters[(int)Parameter::LowPassEnabled] = 1.0;
			parameters[(int)Parameter::LowShelfEnabled] = 0.0;
			parameters[(int)Parameter::HighShelfEnabled] = 0.0;
			parameters[(int)Parameter::CutoffEnabled] = 1.0;
			parameters[(int)Parameter::LateStageTap] = 0.0;
			parameters[(int)Parameter::Interpolation] = 1.0;

			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}

	
		void initFactoryHyperplane()
		{
			//parameters from Hyperplane in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0.1549999862909317;
			parameters[(int)Parameter::PreDelay] = 0.0;
			parameters[(int)Parameter::HighPass] = 0.57999998331069946;
			parameters[(int)Parameter::LowPass] = 0.9100000262260437;
			parameters[(int)Parameter::TapCount] = 0.41499990224838257;
			parameters[(int)Parameter::TapLength] = 0.43999996781349182;
			parameters[(int)Parameter::TapGain] = 1.0;
			parameters[(int)Parameter::TapDecay] = 1.0;
			parameters[(int)Parameter::DiffusionEnabled] = 1.0;
			parameters[(int)Parameter::DiffusionStages] = 0.4285714328289032;
			parameters[(int)Parameter::DiffusionDelay] = 0.27500024437904358;
			parameters[(int)Parameter::DiffusionFeedback] = 0.660000205039978;
			parameters[(int)Parameter::LineCount] = 0.72727274894714355;
			parameters[(int)Parameter::LineDelay] = 0.22500017285346985;
			parameters[(int)Parameter::LineDecay] = 0.794999897480011;
			parameters[(int)Parameter::LateDiffusionEnabled] = 1.0;
			parameters[(int)Parameter::LateDiffusionStages] = 1.0;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.22999951243400574;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.59499990940093994;
			parameters[(int)Parameter::PostLowShelfGain] = 0.95999979972839355;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.23999994993209839;
			parameters[(int)Parameter::PostHighShelfGain] = 0.97500002384185791;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.78499996662139893;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.87999981641769409;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.13499999046325684;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.29000008106231689;
			parameters[(int)Parameter::LineModAmount] = 0.53999996185302734;
			parameters[(int)Parameter::LineModRate] = 0.44999989867210388;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.15999998152256012;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.56000012159347534;
			parameters[(int)Parameter::TapSeed] = 0.00048499999684281647;
			parameters[(int)Parameter::DiffusionSeed] = 0.00020799999765586108;
			parameters[(int)Parameter::DelaySeed] = 0.00034699999378062785;
			parameters[(int)Parameter::PostDiffusionSeed] = 0.00037200000951997936;
			parameters[(int)Parameter::CrossSeed] = 0.800000011920929;
			parameters[(int)Parameter::DryOut] = 0.86500018835067749;
			parameters[(int)Parameter::PredelayOut] = 0.0;
			parameters[(int)Parameter::EarlyOut] = 0.8200000524520874;
			parameters[(int)Parameter::MainOut] = 0.79500007629394531;
			parameters[(int)Parameter::HiPassEnabled] = 1.0;
			parameters[(int)Parameter::LowPassEnabled] = 1.0;
			parameters[(int)Parameter::LowShelfEnabled] = 1.0;
			parameters[(int)Parameter::HighShelfEnabled] = 1.0;
			parameters[(int)Parameter::CutoffEnabled] = 1.0;
			parameters[(int)Parameter::LateStageTap] = 1.0;
			parameters[(int)Parameter::Interpolation] = 0.0;

			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}

		void initFactoryMediumSpace()
		{
			//parameters from Medium Space in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0.0;
			parameters[(int)Parameter::PreDelay] = 0.0;
			parameters[(int)Parameter::HighPass] = 0.0;
			parameters[(int)Parameter::LowPass] = 0.63999992609024048;
			parameters[(int)Parameter::TapCount] = 0.51999980211257935;
			parameters[(int)Parameter::TapLength] = 0.26499992609024048;
			parameters[(int)Parameter::TapGain] = 0.69499999284744263;
			parameters[(int)Parameter::TapDecay] = 1.0;
			parameters[(int)Parameter::DiffusionEnabled] = 1.0;
			parameters[(int)Parameter::DiffusionStages] = 0.8571428656578064;
			parameters[(int)Parameter::DiffusionDelay] = 0.5700000524520874;
			parameters[(int)Parameter::DiffusionFeedback] = 0.76000010967254639;
			parameters[(int)Parameter::LineCount] = 0.18181818723678589;
			parameters[(int)Parameter::LineDelay] = 0.585000216960907;
			parameters[(int)Parameter::LineDecay] = 0.29499980807304382;
			parameters[(int)Parameter::LateDiffusionEnabled] = 1.0;
			parameters[(int)Parameter::LateDiffusionStages] = 0.57142859697341919;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.69499951601028442;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.71499985456466675;
			parameters[(int)Parameter::PostLowShelfGain] = 0.87999987602233887;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.19499993324279785;
			parameters[(int)Parameter::PostHighShelfGain] = 0.72000008821487427;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.520000159740448;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.79999983310699463;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.13499999046325684;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.26000010967254639;
			parameters[(int)Parameter::LineModAmount] = 0.054999928921461105;
			parameters[(int)Parameter::LineModRate] = 0.21499986946582794;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.17999963462352753;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.38000011444091797;
			parameters[(int)Parameter::TapSeed] = 0.0003009999927598983;
			parameters[(int)Parameter::DiffusionSeed] = 0.00018899999849963933;
			parameters[(int)Parameter::DelaySeed] = 0.0001610000035725534;
			parameters[(int)Parameter::PostDiffusionSeed] = 0.00050099997315555811;
			parameters[(int)Parameter::CrossSeed] = 0.7850000262260437;
			parameters[(int)Parameter::DryOut] = 1.0;
			parameters[(int)Parameter::PredelayOut] = 0.0;
			parameters[(int)Parameter::EarlyOut] = 0.699999988079071;
			parameters[(int)Parameter::MainOut] = 0.84499984979629517;
			parameters[(int)Parameter::HiPassEnabled] = 0.0;
			parameters[(int)Parameter::LowPassEnabled] = 1.0;
			parameters[(int)Parameter::LowShelfEnabled] = 1.0;
			parameters[(int)Parameter::HighShelfEnabled] = 0.0;
			parameters[(int)Parameter::CutoffEnabled] = 1.0;
			parameters[(int)Parameter::LateStageTap] = 1.0;
			parameters[(int)Parameter::Interpolation] = 1.0;

			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}

		void initFactoryNoiseInTheHallway()
		{
			//parameters from Noise In The Hallway in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0.0;
			parameters[(int)Parameter::PreDelay] = 0.0;
			parameters[(int)Parameter::HighPass] = 0.0;
			parameters[(int)Parameter::LowPass] = 0.60999995470047;
			parameters[(int)Parameter::TapCount] = 1.0;
			parameters[(int)Parameter::TapLength] = 1.0;
			parameters[(int)Parameter::TapGain] = 0.0;
			parameters[(int)Parameter::TapDecay] = 0.830000102519989;
			parameters[(int)Parameter::DiffusionEnabled] = 1.0;
			parameters[(int)Parameter::DiffusionStages] = 0.28571429848670959;
			parameters[(int)Parameter::DiffusionDelay] = 0.35499998927116394;
			parameters[(int)Parameter::DiffusionFeedback] = 0.62500005960464478;
			parameters[(int)Parameter::LineCount] = 0.63636362552642822;
			parameters[(int)Parameter::LineDelay] = 0.36000004410743713;
			parameters[(int)Parameter::LineDecay] = 0.51000005006790161;
			parameters[(int)Parameter::LateDiffusionEnabled] = 1.0;
			parameters[(int)Parameter::LateDiffusionStages] = 0.0;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.62999987602233887;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.49000000953674316;
			parameters[(int)Parameter::PostLowShelfGain] = 0.0;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.0;
			parameters[(int)Parameter::PostHighShelfGain] = 0.77499985694885254;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.58000004291534424;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.0;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.0;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.0;
			parameters[(int)Parameter::LineModAmount] = 0.0;
			parameters[(int)Parameter::LineModRate] = 0.0;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.0;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.0;
			parameters[(int)Parameter::TapSeed] = 0.0001140000022132881;
			parameters[(int)Parameter::DiffusionSeed] = 0.000155999994603917;
			parameters[(int)Parameter::DelaySeed] = 0.00018099999579135329;
			parameters[(int)Parameter::PostDiffusionSeed] = 8.4999999671708792E-05;
			parameters[(int)Parameter::CrossSeed] = 1.0;
			parameters[(int)Parameter::DryOut] = 0.0;
			parameters[(int)Parameter::PredelayOut] = 0.0;
			parameters[(int)Parameter::EarlyOut] = 0.64500010013580322;
			parameters[(int)Parameter::MainOut] = 0.63000005483627319;
			parameters[(int)Parameter::HiPassEnabled] = 0.0;
			parameters[(int)Parameter::LowPassEnabled] = 1.0;
			parameters[(int)Parameter::LowShelfEnabled] = 0.0;
			parameters[(int)Parameter::HighShelfEnabled] = 1.0;
			parameters[(int)Parameter::CutoffEnabled] = 0.0;
			parameters[(int)Parameter::LateStageTap] = 0.0;
			parameters[(int)Parameter::Interpolation] = 1.0;

			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}

		void initFactoryRubiKaFields()
		{
			//parameters from Rubi-Ka Fields in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0.32499998807907104;
			parameters[(int)Parameter::PreDelay] = 0.0;
			parameters[(int)Parameter::HighPass] = 0.0;
			parameters[(int)Parameter::LowPass] = 0.8899998664855957;
			parameters[(int)Parameter::TapCount] = 0.51999980211257935;
			parameters[(int)Parameter::TapLength] = 1.0;
			parameters[(int)Parameter::TapGain] = 0.90000003576278687;
			parameters[(int)Parameter::TapDecay] = 1.0;
			parameters[(int)Parameter::DiffusionEnabled] = 1.0;
			parameters[(int)Parameter::DiffusionStages] = 0.8571428656578064;
			parameters[(int)Parameter::DiffusionDelay] = 0.5700000524520874;
			parameters[(int)Parameter::DiffusionFeedback] = 0.76000010967254639;
			parameters[(int)Parameter::LineCount] = 0.27272728085517883;
			parameters[(int)Parameter::LineDelay] = 0.68500018119812012;
			parameters[(int)Parameter::LineDecay] = 0.82999974489212036;
			parameters[(int)Parameter::LateDiffusionEnabled] = 1.0;
			parameters[(int)Parameter::LateDiffusionStages] = 0.71428573131561279;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.69499951601028442;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.71499985456466675;
			parameters[(int)Parameter::PostLowShelfGain] = 0.87999987602233887;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.19499993324279785;
			parameters[(int)Parameter::PostHighShelfGain] = 0.72000008821487427;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.520000159740448;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.79999983310699463;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.13499999046325684;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.26000010967254639;
			parameters[(int)Parameter::LineModAmount] = 0.054999928921461105;
			parameters[(int)Parameter::LineModRate] = 0.21499986946582794;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.32499963045120239;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.35500010848045349;
			parameters[(int)Parameter::TapSeed] = 0.0003009999927598983;
			parameters[(int)Parameter::DiffusionSeed] = 0.00018899999849963933;
			parameters[(int)Parameter::DelaySeed] = 0.0001610000035725534;
			parameters[(int)Parameter::PostDiffusionSeed] = 0.00050099997315555811;
			parameters[(int)Parameter::CrossSeed] = 0.43000003695487976;
			parameters[(int)Parameter::DryOut] = 0.88499999046325684;
			parameters[(int)Parameter::PredelayOut] = 0.0;
			parameters[(int)Parameter::EarlyOut] = 0.0;
			parameters[(int)Parameter::MainOut] = 0.90999990701675415;
			parameters[(int)Parameter::HiPassEnabled] = 0.0;
			parameters[(int)Parameter::LowPassEnabled] = 0.0;
			parameters[(int)Parameter::LowShelfEnabled] = 0.0;
			parameters[(int)Parameter::HighShelfEnabled] = 0.0;
			parameters[(int)Parameter::CutoffEnabled] = 1.0;
			parameters[(int)Parameter::LateStageTap] = 1.0;
			parameters[(int)Parameter::Interpolation] = 0.0;

			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}

		
		void initFactorySmallRoom()
		{
			//parameters from Small Room in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0.0;
			parameters[(int)Parameter::PreDelay] = 0.0;
			parameters[(int)Parameter::HighPass] = 0.0;
			parameters[(int)Parameter::LowPass] = 0.755000114440918;
			parameters[(int)Parameter::TapCount] = 0.41499990224838257;
			parameters[(int)Parameter::TapLength] = 0.43999996781349182;
			parameters[(int)Parameter::TapGain] = 0.87999999523162842;
			parameters[(int)Parameter::TapDecay] = 1.0;
			parameters[(int)Parameter::DiffusionEnabled] = 1.0;
			parameters[(int)Parameter::DiffusionStages] = 0.71428573131561279;
			parameters[(int)Parameter::DiffusionDelay] = 0.335000216960907;
			parameters[(int)Parameter::DiffusionFeedback] = 0.660000205039978;
			parameters[(int)Parameter::LineCount] = 0.18181818723678589;
			parameters[(int)Parameter::LineDelay] = 0.51000016927719116;
			parameters[(int)Parameter::LineDecay] = 0.29999998211860657;
			parameters[(int)Parameter::LateDiffusionEnabled] = 1.0;
			parameters[(int)Parameter::LateDiffusionStages] = 0.4285714328289032;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.22999951243400574;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.59499990940093994;
			parameters[(int)Parameter::PostLowShelfGain] = 0.87999987602233887;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.19499993324279785;
			parameters[(int)Parameter::PostHighShelfGain] = 0.875;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.59000009298324585;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.79999983310699463;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.13499999046325684;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.29000008106231689;
			parameters[(int)Parameter::LineModAmount] = 0.18999995291233063;
			parameters[(int)Parameter::LineModRate] = 0.22999987006187439;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.1249999925494194;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.28500008583068848;
			parameters[(int)Parameter::TapSeed] = 0.00048499999684281647;
			parameters[(int)Parameter::DiffusionSeed] = 0.00020799999765586108;
			parameters[(int)Parameter::DelaySeed] = 0.00033499998971819878;
			parameters[(int)Parameter::PostDiffusionSeed] = 0.00037200000951997936;
			parameters[(int)Parameter::CrossSeed] = 0.42500001192092896;
			parameters[(int)Parameter::DryOut] = 1.0;
			parameters[(int)Parameter::PredelayOut] = 0.0;
			parameters[(int)Parameter::EarlyOut] = 0.8599998950958252;
			parameters[(int)Parameter::MainOut] = 0.90500003099441528;
			parameters[(int)Parameter::HiPassEnabled] = 0.0;
			parameters[(int)Parameter::LowPassEnabled] = 1.0;
			parameters[(int)Parameter::LowShelfEnabled] = 0.0;
			parameters[(int)Parameter::HighShelfEnabled] = 0.0;
			parameters[(int)Parameter::CutoffEnabled] = 0.0;
			parameters[(int)Parameter::LateStageTap] = 1.0;
			parameters[(int)Parameter::Interpolation] = 1.0;
			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}

		void initFactory90sAreBack()
		{
			//parameters from The 90s Are Back in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0;
			parameters[(int)Parameter::PreDelay] = 0;
			parameters[(int)Parameter::HighPass] = 0;
			parameters[(int)Parameter::LowPass] = 0.6750001311302185;
			parameters[(int)Parameter::TapCount] = 0;
			parameters[(int)Parameter::TapLength] = 1;
			parameters[(int)Parameter::TapGain] = 0;
			parameters[(int)Parameter::TapDecay] = 0.8650001287460327;
			parameters[(int)Parameter::DiffusionEnabled] = 1;
			parameters[(int)Parameter::DiffusionStages] = 0.5714285969734192;
			parameters[(int)Parameter::DiffusionDelay] = 0.7100000381469727;
			parameters[(int)Parameter::DiffusionFeedback] = 0.5450003147125244;
			parameters[(int)Parameter::LineCount] = 0.7272727489471436;
			parameters[(int)Parameter::LineDelay] = 0.6849998831748962;
			parameters[(int)Parameter::LineDecay] = 0.6300000548362732;
			parameters[(int)Parameter::LateDiffusionEnabled] = 0;
			parameters[(int)Parameter::LateDiffusionStages] = 0.2857142984867096;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.5449999570846558;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.6599999666213989;
			parameters[(int)Parameter::PostLowShelfGain] = 0.5199999213218689;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.31499990820884705;
			parameters[(int)Parameter::PostHighShelfGain] = 0.8349999189376831;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.705000102519989;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.7349998354911804;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.824999988079071;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.4050004780292511;
			parameters[(int)Parameter::LineModAmount] = 0.6300000548362732;
			parameters[(int)Parameter::LineModRate] = 0.3199999928474426;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.619999885559082;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.30000022053718567;
			parameters[(int)Parameter::TapSeed] = 0.0011500000255182385;
			parameters[(int)Parameter::DiffusionSeed] = 0.00018899999849963933;
			parameters[(int)Parameter::DelaySeed] = 0.0003370000049471855;
			parameters[(int)Parameter::PostDiffusionSeed] = 0.0005009999731555581;
			parameters[(int)Parameter::CrossSeed] = 0.7950000166893005;
			parameters[(int)Parameter::DryOut] = 0.9449997544288635;
			parameters[(int)Parameter::PredelayOut] = 0;
			parameters[(int)Parameter::EarlyOut] = 0.7250000238418579;
			parameters[(int)Parameter::MainOut] = 0.6050001382827759;
			parameters[(int)Parameter::HiPassEnabled] = 0;
			parameters[(int)Parameter::LowPassEnabled] = 1;
			parameters[(int)Parameter::LowShelfEnabled] = 0;
			parameters[(int)Parameter::HighShelfEnabled] = 1;
			parameters[(int)Parameter::CutoffEnabled] = 0;
			parameters[(int)Parameter::LateStageTap] = 1;
			parameters[(int)Parameter::Interpolation] = 1;
			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}

		void initFactoryThroughTheLookingGlass()
		{
			//parameters from Through The Looking Glass in
			//https://github.com/ValdemarOrn/CloudSeed/tree/master/Factory%20Programs
			parameters[(int)Parameter::InputMix] = 0.0;
			parameters[(int)Parameter::PreDelay] = 0.0;
			parameters[(int)Parameter::HighPass] = 0.0;
			parameters[(int)Parameter::LowPass] = 0.74000012874603271;
			parameters[(int)Parameter::TapCount] = 1.0;
			parameters[(int)Parameter::TapLength] = 1.0;
			parameters[(int)Parameter::TapGain] = 1.0;
			parameters[(int)Parameter::TapDecay] = 0.71000003814697266;
			parameters[(int)Parameter::DiffusionEnabled] = 1.0;
			parameters[(int)Parameter::DiffusionStages] = 1.0;
			parameters[(int)Parameter::DiffusionDelay] = 0.65999996662139893;
			parameters[(int)Parameter::DiffusionFeedback] = 0.76000010967254639;
			parameters[(int)Parameter::LineCount] = 1.0;
			parameters[(int)Parameter::LineDelay] = 0.9100002646446228;
			parameters[(int)Parameter::LineDecay] = 0.80999958515167236;
			parameters[(int)Parameter::LateDiffusionEnabled] = 1.0;
			parameters[(int)Parameter::LateDiffusionStages] = 1.0;
			parameters[(int)Parameter::LateDiffusionDelay] = 0.71499955654144287;
			parameters[(int)Parameter::LateDiffusionFeedback] = 0.71999979019165039;
			parameters[(int)Parameter::PostLowShelfGain] = 0.87999987602233887;
			parameters[(int)Parameter::PostLowShelfFrequency] = 0.19499993324279785;
			parameters[(int)Parameter::PostHighShelfGain] = 0.72000008821487427;
			parameters[(int)Parameter::PostHighShelfFrequency] = 0.520000159740448;
			parameters[(int)Parameter::PostCutoffFrequency] = 0.7150002121925354;
			parameters[(int)Parameter::EarlyDiffusionModAmount] = 0.41999998688697815;
			parameters[(int)Parameter::EarlyDiffusionModRate] = 0.30500012636184692;
			parameters[(int)Parameter::LineModAmount] = 0.4649999737739563;
			parameters[(int)Parameter::LineModRate] = 0.3199998140335083;
			parameters[(int)Parameter::LateDiffusionModAmount] = 0.40999993681907654;
			parameters[(int)Parameter::LateDiffusionModRate] = 0.31500011682510376;
			parameters[(int)Parameter::TapSeed] = 0.0003009999927598983;
			parameters[(int)Parameter::DiffusionSeed] = 0.00018899999849963933;
			parameters[(int)Parameter::DelaySeed] = 0.0001610000035725534;
			parameters[(int)Parameter::PostDiffusionSeed] = 0.00050099997315555811;
			parameters[(int)Parameter::CrossSeed] = 1.0;
			parameters[(int)Parameter::DryOut] = 0.0;
			parameters[(int)Parameter::PredelayOut] = 0.0;
			parameters[(int)Parameter::EarlyOut] = 0.0;
			parameters[(int)Parameter::MainOut] = 0.95499974489212036;
			parameters[(int)Parameter::HiPassEnabled] = 0.0;
			parameters[(int)Parameter::LowPassEnabled] = 1.0;
			parameters[(int)Parameter::LowShelfEnabled] = 0.0;
			parameters[(int)Parameter::HighShelfEnabled] = 0.0;
			parameters[(int)Parameter::CutoffEnabled] = 1.0;
			parameters[(int)Parameter::LateStageTap] = 1.0;
			parameters[(int)Parameter::Interpolation] = 1.0;
			for (auto value = 0; value < (int)Parameter::Count; value++)
			{
				SetParameter((Parameter)value, parameters[value]);
			}

		}

		int GetSamplerate()
		{
			return samplerate;
		}

		void SetSamplerate(int samplerate)
		{
			this->samplerate = samplerate;

			channelL.SetSamplerate(samplerate);
			channelR.SetSamplerate(samplerate);
		}

		int GetParameterCount()
		{
			return (int)Parameter::Count;
		}

		float* GetAllParameters()
		{
			return parameters;
		}

		float GetScaledParameter(Parameter param)
		{
			switch (param)
			{
				// Input
			case Parameter::InputMix:                  return P(Parameter::InputMix);
			case Parameter::PreDelay:                  return (int)(P(Parameter::PreDelay) * 1000);

			case Parameter::HighPass:                  return 20 + ValueTables::Get(P(Parameter::HighPass), ValueTables::Response4Oct) * 980;
			case Parameter::LowPass:                   return 400 + ValueTables::Get(P(Parameter::LowPass), ValueTables::Response4Oct) * 19600;

				// Early
			case Parameter::TapCount:                  return 1 + (int)(P(Parameter::TapCount) * (MultitapDiffuser::MaxTaps - 1));
			case Parameter::TapLength:                 return (int)(P(Parameter::TapLength) * 500);
			case Parameter::TapGain:                   return ValueTables::Get(P(Parameter::TapGain), ValueTables::Response2Dec);
			case Parameter::TapDecay:                  return P(Parameter::TapDecay);

			case Parameter::DiffusionEnabled:          return P(Parameter::DiffusionEnabled) < 0.5 ? 0.0 : 1.0;
			case Parameter::DiffusionStages:           return 1 + (int)(P(Parameter::DiffusionStages) * (AllpassDiffuser::MaxStageCount - 0.001));
			case Parameter::DiffusionDelay:            return (int)(10 + P(Parameter::DiffusionDelay) * 90);
			case Parameter::DiffusionFeedback:         return P(Parameter::DiffusionFeedback);

				// Late
			case Parameter::LineCount:                 return 1 + (int)(P(Parameter::LineCount) * 11.999);
			case Parameter::LineDelay:                 return (int)(20.0 + ValueTables::Get(P(Parameter::LineDelay), ValueTables::Response2Dec) * 980);
			case Parameter::LineDecay:                 return 0.05 + ValueTables::Get(P(Parameter::LineDecay), ValueTables::Response3Dec) * 59.95;

			case Parameter::LateDiffusionEnabled:      return P(Parameter::LateDiffusionEnabled) < 0.5 ? 0.0 : 1.0;
			case Parameter::LateDiffusionStages:       return 1 + (int)(P(Parameter::LateDiffusionStages) * (AllpassDiffuser::MaxStageCount - 0.001));
			case Parameter::LateDiffusionDelay:        return (int)(10 + P(Parameter::LateDiffusionDelay) * 90);
			case Parameter::LateDiffusionFeedback:     return P(Parameter::LateDiffusionFeedback);

				// Frequency Response
			case Parameter::PostLowShelfGain:          return ValueTables::Get(P(Parameter::PostLowShelfGain), ValueTables::Response2Dec);
			case Parameter::PostLowShelfFrequency:     return 20 + ValueTables::Get(P(Parameter::PostLowShelfFrequency), ValueTables::Response4Oct) * 980;
			case Parameter::PostHighShelfGain:         return ValueTables::Get(P(Parameter::PostHighShelfGain), ValueTables::Response2Dec);
			case Parameter::PostHighShelfFrequency:    return 400 + ValueTables::Get(P(Parameter::PostHighShelfFrequency), ValueTables::Response4Oct) * 19600;
			case Parameter::PostCutoffFrequency:       return 400 + ValueTables::Get(P(Parameter::PostCutoffFrequency), ValueTables::Response4Oct) * 19600;

				// Modulation
			case Parameter::EarlyDiffusionModAmount:   return P(Parameter::EarlyDiffusionModAmount) * 2.5;
			case Parameter::EarlyDiffusionModRate:     return ValueTables::Get(P(Parameter::EarlyDiffusionModRate), ValueTables::Response2Dec) * 5;
			case Parameter::LineModAmount:             return P(Parameter::LineModAmount) * 2.5;
			case Parameter::LineModRate:               return ValueTables::Get(P(Parameter::LineModRate), ValueTables::Response2Dec) * 5;
			case Parameter::LateDiffusionModAmount:    return P(Parameter::LateDiffusionModAmount) * 2.5;
			case Parameter::LateDiffusionModRate:      return ValueTables::Get(P(Parameter::LateDiffusionModRate), ValueTables::Response2Dec) * 5;

				// Seeds
			case Parameter::TapSeed:                   return (int)std::floor(P(Parameter::TapSeed) * 1000000 + 0.001);
			case Parameter::DiffusionSeed:             return (int)std::floor(P(Parameter::DiffusionSeed) * 1000000 + 0.001);
			case Parameter::DelaySeed:                 return (int)std::floor(P(Parameter::DelaySeed) * 1000000 + 0.001);
			case Parameter::PostDiffusionSeed:         return (int)std::floor(P(Parameter::PostDiffusionSeed) * 1000000 + 0.001);

				// Output
			case Parameter::CrossSeed:                 return P(Parameter::CrossSeed);

			case Parameter::DryOut:                    return ValueTables::Get(P(Parameter::DryOut), ValueTables::Response2Dec);
			case Parameter::PredelayOut:               return ValueTables::Get(P(Parameter::PredelayOut), ValueTables::Response2Dec);
			case Parameter::EarlyOut:                  return ValueTables::Get(P(Parameter::EarlyOut), ValueTables::Response2Dec);
			case Parameter::MainOut:                   return ValueTables::Get(P(Parameter::MainOut), ValueTables::Response2Dec);

				// Switches
			case Parameter::HiPassEnabled:             return P(Parameter::HiPassEnabled) < 0.5 ? 0.0 : 1.0;
			case Parameter::LowPassEnabled:            return P(Parameter::LowPassEnabled) < 0.5 ? 0.0 : 1.0;
			case Parameter::LowShelfEnabled:           return P(Parameter::LowShelfEnabled) < 0.5 ? 0.0 : 1.0;
			case Parameter::HighShelfEnabled:          return P(Parameter::HighShelfEnabled) < 0.5 ? 0.0 : 1.0;
			case Parameter::CutoffEnabled:             return P(Parameter::CutoffEnabled) < 0.5 ? 0.0 : 1.0;
			case Parameter::LateStageTap:			   return P(Parameter::LateStageTap) < 0.5 ? 0.0 : 1.0;

				// Effects
			case Parameter::Interpolation:			   return P(Parameter::Interpolation) < 0.5 ? 0.0 : 1.0;

			default: return 0.0;
			}

			return 0.0;
		}

		void SetParameter(Parameter param, float value)
		{
			parameters[(int)param] = value;
			auto scaled = GetScaledParameter(param);
			
			channelL.SetParameter(param, scaled);
			channelR.SetParameter(param, scaled);
		}

		

		void ClearBuffers()
		{
			channelL.ClearBuffers();
			channelR.ClearBuffers();
		}

		void Process(float* input, float* output, int bufferSize)
		{
			auto len = bufferSize;
			auto cm = GetScaledParameter(Parameter::InputMix) * 0.5;
			auto cmi = (1 - cm);

			for (int i = 0; i < len; i++)
			{
				leftChannelIn[i] = input[i*2] * cmi + input[i*2+1] * cm;
				rightChannelIn[i] = input[i*2+1] * cmi + input[i*2] * cm;
			}

			channelL.Process(leftChannelIn, len);
			channelR.Process(rightChannelIn, len);
			auto leftOut = channelL.GetOutput();
			auto rightOut = channelR.GetOutput();

			for (int i = 0; i < len; i++)
			{
				output[i*2] = leftOut[i];
				output[i*2+1] = rightOut[i];
			}
		}
		
	private:
		float P(Parameter para)
		{
			auto idx = (int)para;
			return idx >= 0 && idx < (int)Parameter::Count ? parameters[idx] : 0.0;
		}
	};
}
#endif
