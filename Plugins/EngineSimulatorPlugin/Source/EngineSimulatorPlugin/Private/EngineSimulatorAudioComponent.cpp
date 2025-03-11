// Fill out your copyright notice in the Description page of Project Settings.


#include "EngineSimulatorAudioComponent.h"

#include "EngineSimulator.h"
#include "EngineSimulatorEngineInterface.h"
#include "Sound/AudioBus.h"
#include "AudioMixerDevice.h"

UEngineSimulatorAudioComponent::UEngineSimulatorAudioComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	SetComponentTickEnabled(true);
	bAutoActivate = true;

	bAutomaticallySetEngineComponent = true;

	bOutputToAudioBus = false;
}

void UEngineSimulatorAudioComponent::OnComponentCreated()
{
	Super::OnComponentCreated();

	if (bOutputToAudioBus)
	{
		AudioBus = NewObject<UAudioBus>(this);
		FSoundSourceBusSendInfo BusSendInfo;
		BusSendInfo.AudioBus = AudioBus;
		if (BusSends.Num() == 0)
		{
			BusSends.Add(BusSendInfo);
		}
		else
		{
			BusSends[0] = BusSendInfo;
		}
	}
}

bool UEngineSimulatorAudioComponent::Init(int32& InSampleRate)
{
	InSampleRate = 44100; // Set desired sample rate here
	NumChannels = 1;      // Mono audio

	return true;
}

void UEngineSimulatorAudioComponent::BeginPlay()
{
	Super::BeginPlay();

	if (bAutomaticallySetEngineComponent)
	{
		if (AActor* Owner = GetOwner())
		{
			TArray<UActorComponent*> Components;
			Owner->GetComponents(Components);

			EngineComponent = nullptr;

			for (UActorComponent* Component : Components)
			{
				if (Component && Component->GetClass()->ImplementsInterface(UEngineSimulatorEngineInterface::StaticClass()))
				{
					IEngineSimulatorEngineInterface* EngineInterface = Cast<IEngineSimulatorEngineInterface>(Component);
					if (EngineInterface)
					{
						EngineComponent = Component;
						EngineSimulator = EngineInterface->GetEngineSimulator();
						break;
					}
				}
			}

			if (!EngineSimulator.IsValid())
			{
				UE_LOG(LogTemp, Error, TEXT("Could not find a valid EngineSimulator instance."));
			}
		}
		else
		{
			UE_LOG(LogTemp, Error, TEXT("UEngineSimulatorAudioComponent has no owner!"));
		}

		SetOutputToBusOnly(bOutputToAudioBus);

		if (bOutputToAudioBus && AudioBus == nullptr)
		{
			AudioBus = NewObject<UAudioBus>(this);
			FSoundSourceBusSendInfo BusSendInfo;
			BusSendInfo.AudioBus = AudioBus;

			if (BusSends.Num() == 0)
			{
				BusSends.Add(BusSendInfo);
			}
			else
			{
				BusSends[0] = BusSendInfo;
			}
			SetAudioBusSendPostEffect(AudioBus, 1.0f);
		}
	}
}

int32 UEngineSimulatorAudioComponent::OnGenerateAudio(float* OutAudio, int32 NumSamples)
{
	if (EngineSimulator.IsValid())
	{
		int32 GeneratedSound = EngineSimulator->GenerateAudio(OutAudio, NumSamples);

		if (GeneratedSound < NumSamples)
		{
			UE_LOG(LogTemp, Warning, TEXT("UEngineSimulatorAudioComponent::OnGenerateAudio: Buffer Underrun, not enough samples. Is EngineSimulator ticking fast enough?"))
		}

		return GeneratedSound;
	}

	return NumSamples;
}

void UEngineSimulatorAudioComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	EngineSimulator = EngineComponent != nullptr ? EngineComponent->GetEngineSimulator() : nullptr;
}