/**
 * @file mainwindow.h
 * @brief Manages the content of the GUI window.
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Konrad Armbrecht <konrad.armbrecht@tha.de>
      Technische Hochschule Augsburg, University of Applied Sciences

  Description: This file contains the implementation of the main GUI window
  for the piconut simulator. It is responsible for displaying the framebuffer
  content as an image, handling window resizing while maintaining the correct
  aspect ratio, and managing the window state.

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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QResizeEvent>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

enum GUI_ACTION {
    SETTINGS_AUDIO
};

/**
 * @class mainwindow
 * @author Konrad Armbrecht
 *
 * This class provides a Qt-based GUI window that displays the assembled output image.
 * It is responsible for filling the QLabel with the output image.
 *
 * The GUI adapts dynamically to changes in window size while preserving the
 * correct aspect ratio. Additionally, it allows for simulation control, as
 * the simulator is terminated when the main window is closed.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Constructor.
     * @param parent The parent widget (default: nullptr).
     *
     * Initializes the UI components and sets up the image display area.
     */
    MainWindow(QWidget* parent = nullptr);

    /**
     * @brief Destructor.
     */
    virtual ~MainWindow();

    /**
     * @brief Updates the GUI with the latest framebuffer content.
     * @param framebuffer Pointer to the framebuffer data.
     * @param framebuffer_size Size of the framebuffer in pixel.
     *
     * Converts the framebuffer content into a `QPixmap` and displays it
     * in the GUI. The image adapts to the QLabel size dynamically.
     */
    void image_to_gui(uint32_t* framebuffer, uint32_t framebuffer_size);
    
    /**
     * @brief Check if the main window is open.
     * @return True if the main window is open, otherwise false.
     */
    bool is_mainwindow_open() const;

    std::function<void(GUI_ACTION)> triggerCallback = nullptr;

protected:
    /**
     * @brief Handles window resize events.
     * @param event The resize event.
     *
     * Ensures that the window maintains the correct aspect ratio based
     * on the framebuffer resolution.
     */
    void resizeEvent(QResizeEvent* event) override;

    void closeEvent(QCloseEvent *event) override;

private:
    Ui::MainWindow* ui;          ///< Pointer to the UI components.
    QImage output_image;         ///< Stores the converted framebuffer image.
    bool mainwindow_open = true; ///< Tracks whether the main window is open.
};
#endif // MAINWINDOW_H
