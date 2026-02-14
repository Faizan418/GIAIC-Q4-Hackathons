import { NextRequest, NextResponse } from 'next/server';
import { TaskService } from '@/lib/task-service';

/**
 * PUT /api/tasks/:id
 * Update an existing task
 */
export async function PUT(request: NextRequest, { params }: { params: Promise<{ id: string }> }) {
  try {
    const resolvedParams = await params;
    const { id } = resolvedParams;

    if (!id || id.length === 0) {
      return NextResponse.json(
        {
          success: false,
          error: 'Task ID is required'
        },
        { status: 400 }
      );
    }

    // Parse request body
    const body = await request.json();

    // For public access model, using mock user ID
    const userId = 'public-user';

    // Update task using the service
    const updatedTask = await TaskService.update(id, body);

    return NextResponse.json({
      success: true,
      data: updatedTask
    });
  } catch (error) {
    console.error('Error updating task:', error);

    return NextResponse.json(
      {
        success: false,
        error: 'Failed to update task',
        message: error instanceof Error ? error.message : 'Unknown error occurred'
      },
      { status: 500 }
    );
  }
}

/**
 * PATCH /api/tasks/:id/complete
 * Toggle task completion status
 */
export async function PATCH(request: NextRequest, { params }: { params: Promise<{ id: string }> }) {
  try {
    const resolvedParams = await params;
    const { id } = resolvedParams;

    if (!id || id.length === 0) {
      return NextResponse.json(
        {
          success: false,
          error: 'Task ID is required'
        },
        { status: 400 }
      );
    }

    // Parse request body
    const body = await request.json();
    const { completed } = body;

    if (completed === undefined) {
      return NextResponse.json(
        {
          success: false,
          error: 'Completion status is required'
        },
        { status: 400 }
      );
    }

    // For public access model, using mock user ID
    const userId = 'public-user';

    // Update task completion status using the service
    const updatedTask = await TaskService.toggleCompletion(id, completed);

    return NextResponse.json({
      success: true,
      data: updatedTask
    });
  } catch (error) {
    console.error('Error updating task completion:', error);

    return NextResponse.json(
      {
        success: false,
        error: 'Failed to update task completion',
        message: error instanceof Error ? error.message : 'Unknown error occurred'
      },
      { status: 500 }
    );
  }
}

/**
 * DELETE /api/tasks/:id
 * Delete a task
 */
export async function DELETE(request: NextRequest, { params }: { params: Promise<{ id: string }> }) {
  try {
    const resolvedParams = await params;
    const { id } = resolvedParams;

    if (!id || id.length === 0) {
      return NextResponse.json(
        {
          success: false,
          error: 'Task ID is required'
        },
        { status: 400 }
      );
    }

    // For public access model, using mock user ID
    const userId = 'public-user';

    // Delete task using the service
    await TaskService.delete(id);

    return NextResponse.json({
      success: true,
      message: 'Task deleted successfully'
    }, { status: 200 });
  } catch (error) {
    console.error('Error deleting task:', error);

    return NextResponse.json(
      {
        success: false,
        error: 'Failed to delete task',
        message: error instanceof Error ? error.message : 'Unknown error occurred'
      },
      { status: 500 }
    );
  }
}