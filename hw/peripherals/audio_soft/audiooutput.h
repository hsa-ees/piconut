/**
 * @file audiooutput.h
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

#ifndef AUDIOOUTPUT_H
#define AUDIOOUTPUT_H

#include <QAudioSink>
#include <QByteArray>
#include <QComboBox>
#include <QIODevice>
#include <QLabel>
#include <QMainWindow>
#include <QMediaDevices>
#include <QObject>
#include <QPushButton>
#include <QScopedPointer>
#include <QSlider>
#include <QTimer>

#include <math.h>

typedef long long int (*read_func)(char*, long long int, void *ctx);

class Generator : public QIODevice
{
    Q_OBJECT

public:
    Generator(read_func, void* context);

    void start();
    void stop();

    qint64 readData(char *data, qint64 maxlen) override;
    qint64 writeData(const char *data, qint64 len) override;
    qint64 bytesAvailable() const override;
    qint64 size() const override { return m_buffer.size(); }

    void setFrequency(int);

private:
    void generateData(const QAudioFormat &format, qint64 durationUs, int sampleRate);

    read_func read_callback;
    void *context;

private:
    qint64 m_pos = 0;
    QByteArray m_buffer;
    qint64 frequency = 100;
};

class AudioTest : public QMainWindow
{
    Q_OBJECT

public:
    AudioTest(read_func, void* ctx);
    ~AudioTest();

    void startPlayback(int bitrate, bool stereo, uint8_t bits_per_sample, bool signed_samples, bool float_samples);
    void stopPlayback();

private:
    void initializeWindow();

    int bitrate = 44100;
    bool stereo = false;
    bool playbackStarted = false;
    uint8_t bits_per_sample;
    bool signed_samples;
    bool float_samples;

private:
    QMediaDevices *m_devices = nullptr;

    // Owned by layout
    QComboBox *m_deviceBox = nullptr;
    QLabel *m_volumeLabel = nullptr;
    QSlider *m_volumeSlider = nullptr;
    QLabel *m_frequencyLabel = nullptr;
    QSlider *m_frequencySlider = nullptr;

    QScopedPointer<Generator> m_generator;
    QScopedPointer<QAudioSink> m_audioOutput;

    int selectedDeviceIndex = 0;

    read_func read_callback;
    void *context;


private slots:
    void deviceChanged(int index);
    void volumeChanged(int);
    void updateAudioDevices();
};

#endif // AUDIOOUTPUT_H
