import { NextRequest, NextResponse } from 'next/server';
import { TaskService } from '../../../lib/task-service';
import { validateCreateTask } from '../../../lib/validation';

/**
 * GET /api/tasks
 * Fetch all tasks for the authenticated user
 */
export async function GET(request: NextRequest) {
  try {
    // Extract user from request (assuming JWT or session-based auth)
    // For now, using a mock user ID - in real implementation, extract from headers/session
    const url = new URL(request.url);
    const userId = url.searchParams.get('userId') || 'mock-user-id'; // This would come from auth

    // Validate query parameters if any
    const queryParams = {
      userId,
      completed: url.searchParams.get('completed'),
      priority: url.searchParams.get('priority'),
      type: url.searchParams.get('type'),
      search: url.searchParams.get('search'),
      sort: url.searchParams.get('sort'),
      order: url.searchParams.get('order'),
    };

    // In a real implementation, you'd validate these params
    // const validationResult = validateGetTasksQuery(queryParams);
    // if (!validationResult.success) {
    //   return NextResponse.json(
    //     { error: 'Invalid query parameters', details: validationResult.error },
    //     { status: 400 }
    //   );
    // }

    // Fetch tasks using the service
    const tasks = await TaskService.getAll();

    return NextResponse.json({
      success: true,
      data: tasks
    });
  } catch (error) {
    console.error('Error fetching tasks:', error);

    // Return appropriate error response
    return NextResponse.json(
      {
        success: false,
        error: 'Failed to fetch tasks',
        message: error instanceof Error ? error.message : 'Unknown error occurred'
      },
      { status: 500 }
    );
  }
}

/**
 * POST /api/tasks
 * Create a new task
 */
export async function POST(request: NextRequest) {
  try {
    // Parse request body
    const body = await request.json();

    // Validate input using our schema
    const validationResult = validateCreateTask(body);
    if (!validationResult.success) {
      return NextResponse.json(
        {
          success: false,
          error: 'Validation failed',
          details: validationResult.error.flatten()
        },
        { status: 400 }
      );
    }

    const validatedData = validationResult.data;

    // For now, using a mock user ID - in real implementation, extract from auth headers
    const userId = 'public-user'; // Using public access model as per spec

    // Create task using the service
    const newTask = await TaskService.create(validatedData);

    return NextResponse.json({
      success: true,
      data: newTask
    }, { status: 201 });
  } catch (error) {
    console.error('Error creating task:', error);

    return NextResponse.json(
      {
        success: false,
        error: 'Failed to create task',
        message: error instanceof Error ? error.message : 'Unknown error occurred'
      },
      { status: 500 }
    );
  }
}

/**
 * PUT /api/tasks/:id
 * Update an existing task
 */
export async function PUT(request: NextRequest) {
  try {
    const { searchParams } = new URL(request.url);
    const taskId = searchParams.get('id');

    if (!taskId) {
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

    // Validate input using our schema
    // For update, we need to validate against update schema
    const validatedData = { ...body }; // In a real app, we'd validate with updateTaskSchema

    // For public access model, using mock user ID
    const userId = 'public-user';

    // Update task using the service
    const updatedTask = await TaskService.update(taskId, validatedData);

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
 * DELETE /api/tasks/:id
 * Delete a task
 */
export async function DELETE(request: NextRequest) {
  try {
    const { searchParams } = new URL(request.url);
    const taskId = searchParams.get('id');

    if (!taskId) {
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
    await TaskService.delete(taskId);

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