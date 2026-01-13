/**
 * @file audiooutput.cpp
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Daniel Dakhno <Daniel.Dakhno1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_audio for simulation ONLY

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/

#include "audiooutput.h"

#include <QAudioDevice>
#include <QAudioSink>
#include <QAudioFormat>
#include <QDebug>
#include <QVBoxLayout>
#include <QtEndian>
#include <QtMath>

Generator::Generator(read_func read_callback, void *context) : read_callback(read_callback), context(context)
{
}

void Generator::start()
{
    open(QIODevice::ReadOnly);
}

void Generator::stop()
{
    m_pos = 0;
    close();
}

void Generator::setFrequency(int frequency)
{
    this->frequency = frequency;
}

qint64 Generator::readData(char *data, qint64 len)
{
    return this->read_callback(data, len, this->context);
}

qint64 Generator::writeData(const char *data, qint64 len)
{
    Q_UNUSED(data);
    Q_UNUSED(len);

    return 0;
}

qint64 Generator::bytesAvailable() const
{
    return 0xff;
}

AudioTest::AudioTest(read_func read_callback, void *context) : m_devices(new QMediaDevices(this)), read_callback(read_callback), context(context)
{
    initializeWindow();
}

AudioTest::~AudioTest()
{
}

void AudioTest::initializeWindow()
{
    QWidget *window = new QWidget;
    QVBoxLayout *layout = new QVBoxLayout;

    m_deviceBox = new QComboBox(this);
    const QAudioDevice &defaultDeviceInfo = m_devices->defaultAudioOutput();
    m_deviceBox->addItem(defaultDeviceInfo.description(), QVariant::fromValue(defaultDeviceInfo));
    for (auto &deviceInfo : m_devices->audioOutputs()) {
        if (deviceInfo != defaultDeviceInfo)
            m_deviceBox->addItem(deviceInfo.description(), QVariant::fromValue(deviceInfo));
    }
    connect(m_deviceBox, QOverload<int>::of(&QComboBox::activated), this,
            &AudioTest::deviceChanged);
    connect(m_devices, &QMediaDevices::audioOutputsChanged, this, &AudioTest::updateAudioDevices);
    layout->addWidget(m_deviceBox);

    QHBoxLayout *volumeBox = new QHBoxLayout;
    m_volumeLabel = new QLabel;
    m_volumeLabel->setText(tr("Volume:"));
    m_volumeSlider = new QSlider(Qt::Horizontal);
    m_volumeSlider->setMinimum(0);
    m_volumeSlider->setMaximum(100);
    m_volumeSlider->setSingleStep(10);
    connect(m_volumeSlider, &QSlider::valueChanged, this, &AudioTest::volumeChanged);
    volumeBox->addWidget(m_volumeLabel);
    volumeBox->addWidget(m_volumeSlider);
    layout->addLayout(volumeBox);

    window->setLayout(layout);

    setCentralWidget(window);
    window->show();
}

void AudioTest::startPlayback(int bitrate, bool stereo, uint8_t bits_per_sample, bool signed_samples, bool float_samples)
{
    this->bitrate = bitrate;
    this->stereo = stereo;
    this->bits_per_sample = bits_per_sample;
    this->signed_samples = signed_samples;
    this->float_samples = float_samples;
    QAudioDevice deviceInfo = m_deviceBox->itemData(selectedDeviceIndex).value<QAudioDevice>();

    QAudioFormat format = deviceInfo.preferredFormat();
    // could make this adjustable
    format.setChannelConfig(stereo ? QAudioFormat::ChannelConfigStereo : QAudioFormat::ChannelConfigMono);

    if(float_samples) {
        format.setSampleFormat(QAudioFormat::SampleFormat::Float);
    }
    else{
        switch (bits_per_sample)
        {
        case 8:
            format.setSampleFormat(QAudioFormat::SampleFormat::UInt8);
            break;
        case 16:
            format.setSampleFormat(QAudioFormat::SampleFormat::Int16);
            break;
        case 32:
            format.setSampleFormat(QAudioFormat::SampleFormat::Int32);
            break;
        }
    }
    format.setSampleRate(bitrate);

    m_generator.reset(new Generator(this->read_callback, this->context));
    m_audioOutput.reset(new QAudioSink(deviceInfo, format));
    m_audioOutput->setVolume(0.1);
    m_generator->start();

    qreal initialVolume = QAudio::convertVolume(m_audioOutput->volume(), QAudio::LinearVolumeScale,
                                                QAudio::LogarithmicVolumeScale);
    // initialVolume = 0.5;
    m_volumeSlider->setValue(qRound(initialVolume * 100));

    m_audioOutput->reset();

    m_audioOutput->start(m_generator.data());

    playbackStarted = true;
}

void AudioTest::stopPlayback(){
    m_generator->stop();
    m_audioOutput->stop();
    m_audioOutput->disconnect(this);

    playbackStarted = false;
}

void AudioTest::deviceChanged(int index)
{
    selectedDeviceIndex = index;
    if(!playbackStarted) {
        return;
    }
    stopPlayback();
    startPlayback(this->bitrate, this->stereo, this->bits_per_sample, this->signed_samples, this->float_samples);
}

void AudioTest::volumeChanged(int value)
{
    qreal linearVolume = QAudio::convertVolume(value / qreal(100), QAudio::LogarithmicVolumeScale,
                                               QAudio::LinearVolumeScale);

    m_audioOutput->setVolume(linearVolume);
}

void AudioTest::updateAudioDevices()
{
    m_deviceBox->clear();
    const QList<QAudioDevice> devices = m_devices->audioOutputs();
    for (const QAudioDevice &deviceInfo : devices)
        m_deviceBox->addItem(deviceInfo.description(), QVariant::fromValue(deviceInfo));
}

#include "audiooutput_moc.cpp"
